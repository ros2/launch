# Copyright 2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Module for the ExecuteProcess action."""

import asyncio
import io
import os
import platform
import shlex
import signal
import threading
import traceback
from typing import Any  # noqa: F401
from typing import Callable
from typing import cast
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Text
from typing import Tuple  # noqa: F401
from typing import Union

import launch.logging

from osrf_pycommon.process_utils import async_execute_process
from osrf_pycommon.process_utils import AsyncSubprocessProtocol

from .emit_event import EmitEvent
from .opaque_function import OpaqueFunction
from .timer_action import TimerAction

from ..action import Action
from ..conditions import evaluate_condition_expression
from ..event import Event
from ..event_handler import EventHandler
from ..event_handlers import OnProcessExit
from ..event_handlers import OnProcessIO
from ..event_handlers import OnShutdown
from ..events import matches_action
from ..events import Shutdown
from ..events.process import ProcessExited
from ..events.process import ProcessIO
from ..events.process import ProcessStarted
from ..events.process import ProcessStderr
from ..events.process import ProcessStdin
from ..events.process import ProcessStdout
from ..events.process import ShutdownProcess
from ..events.process import SignalProcess
from ..frontend import Entity
from ..frontend import expose_action
from ..frontend import Parser
from ..launch_context import LaunchContext
from ..launch_description import LaunchDescription
from ..launch_description_entity import LaunchDescriptionEntity
from ..some_actions_type import SomeActionsType
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution  # noqa: F401
from ..substitutions import LaunchConfiguration
from ..substitutions import PythonExpression
from ..substitutions import TextSubstitution
from ..utilities import create_future
from ..utilities import is_a_subclass
from ..utilities import normalize_to_list_of_substitutions
from ..utilities import perform_substitutions

_global_process_counter_lock = threading.Lock()
_global_process_counter = 0  # in Python3, this number is unbounded (no rollover)


@expose_action('executable')
class ExecuteProcess(Action):
    """Action that begins executing a process and sets up event handlers for the process."""

    def __init__(
        self,
        *,
        cmd: Iterable[SomeSubstitutionsType],
        name: Optional[SomeSubstitutionsType] = None,
        cwd: Optional[SomeSubstitutionsType] = None,
        env: Optional[Dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
        additional_env: Optional[Dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
        shell: bool = False,
        sigterm_timeout: SomeSubstitutionsType = LaunchConfiguration(
            'sigterm_timeout', default=5),
        sigkill_timeout: SomeSubstitutionsType = LaunchConfiguration(
            'sigkill_timeout', default=5),
        emulate_tty: bool = False,
        prefix: Optional[SomeSubstitutionsType] = None,
        output: Text = 'log',
        output_format: Text = '[{this.name}] {line}',
        log_cmd: bool = False,
        on_exit: Optional[Union[
            SomeActionsType,
            Callable[[ProcessExited, LaunchContext], Optional[SomeActionsType]]
        ]] = None,
        respawn: bool = False,
        respawn_delay: Optional[float] = None,
        **kwargs
    ) -> None:
        """
        Construct an ExecuteProcess action.

        Many arguments are passed eventually to :class:`subprocess.Popen`, so
        see the documentation for the class for additional details.

        This action, once executed, registers several event handlers for
        various process related events and will also emit events asynchronously
        when certain events related to the process occur.

        Handled events include:

        - launch.events.process.ShutdownProcess:

          - begins standard shutdown procedure for a running executable

        - launch.events.process.SignalProcess:

          - passes the signal provided by the event to the running process

        - launch.events.process.ProcessStdin:

          - passes the text provided by the event to the stdin of the process

        - launch.events.Shutdown:

          - same as ShutdownProcess

        Emitted events include:

        - launch.events.process.ProcessStarted:

            - emitted when the process starts

        - launch.events.process.ProcessExited:

            - emitted when the process exits
            - event contains return code

        - launch.events.process.ProcessStdout and launch.events.process.ProcessStderr:

            - emitted when the process produces data on either the stdout or stderr pipes
            - event contains the data from the pipe

        Note that output is just stored in this class and has to be properly
        implemented by the event handlers for the process's ProcessIO events.

        :param: cmd a list where the first item is the executable and the rest
            are arguments to the executable, each item may be a string or a
            list of strings and Substitutions to be resolved at runtime
        :param: cwd the directory in which to run the executable
        :param: name the label used to represent the process, as a string or a
            Substitution to be resolved at runtime, defaults to the basename of
            the executable
        :param: env dictionary of environment variables to be used, starting from
            a clean environment. If 'None', the current environment is used.
        :param: additional_env dictionary of environment variables to be added.
            If 'env' was None, they are added to the current environment.
            If not, 'env' is updated with additional_env.
        :param: shell if True, a shell is used to execute the cmd
        :param: sigterm_timeout time until shutdown should escalate to SIGTERM,
            as a string or a list of strings and Substitutions to be resolved
            at runtime, defaults to the LaunchConfiguration called
            'sigterm_timeout'
        :param: sigkill_timeout time until escalating to SIGKILL after SIGTERM,
            as a string or a list of strings and Substitutions to be resolved
            at runtime, defaults to the LaunchConfiguration called
            'sigkill_timeout'
        :param: emulate_tty emulate a tty (terminal), defaults to False, but can
            be overridden with the LaunchConfiguration called 'emulate_tty',
            the value of which is evaluated as true or false according to
            :py:func:`evaluate_condition_expression`.
            Throws :py:exception:`InvalidConditionExpressionError` if the
            'emulate_tty' configuration does not represent a boolean.
        :param: prefix a set of commands/arguments to preceed the cmd, used for
            things like gdb/valgrind and defaults to the LaunchConfiguration
            called 'launch-prefix'
        :param: output configuration for process output logging. Defaults to 'log'
            i.e. log both stdout and stderr to launch main log file and stderr to
            the screen.
            Overridden externally by the OVERRIDE_LAUNCH_PROCESS_OUTPUT envvar value.
            See `launch.logging.get_output_loggers()` documentation for further
            reference on all available options.
        :param: output_format for logging each output line, supporting `str.format()`
            substitutions with the following keys in scope: `line` to reference the raw
            output line and `this` to reference this action instance.
        :param: log_cmd if True, prints the final cmd before executing the
            process, which is useful for debugging when substitutions are
            involved.
        :param: on_exit list of actions to execute upon process exit.
        :param: respawn if 'True', relaunch the process that abnormally died.
            Defaults to 'False'.
        :param: respawn_delay a delay time to relaunch the died process if respawn is 'True'.
        """
        super().__init__(**kwargs)
        self.__cmd = [normalize_to_list_of_substitutions(x) for x in cmd]
        self.__name = name if name is None else normalize_to_list_of_substitutions(name)
        self.__cwd = cwd if cwd is None else normalize_to_list_of_substitutions(cwd)
        self.__env = None  # type: Optional[List[Tuple[List[Substitution], List[Substitution]]]]
        if env is not None:
            self.__env = []
            for key, value in env.items():
                self.__env.append((
                    normalize_to_list_of_substitutions(key),
                    normalize_to_list_of_substitutions(value)))
        self.__additional_env: Optional[List[Tuple[List[Substitution], List[Substitution]]]] = None
        if additional_env is not None:
            self.__additional_env = []
            for key, value in additional_env.items():
                self.__additional_env.append((
                    normalize_to_list_of_substitutions(key),
                    normalize_to_list_of_substitutions(value)))
        self.__shell = shell
        self.__sigterm_timeout = normalize_to_list_of_substitutions(sigterm_timeout)
        self.__sigkill_timeout = normalize_to_list_of_substitutions(sigkill_timeout)
        self.__emulate_tty = emulate_tty
        self.__prefix = normalize_to_list_of_substitutions(
            LaunchConfiguration('launch-prefix', default='') if prefix is None else prefix
        )
        self.__output = os.environ.get('OVERRIDE_LAUNCH_PROCESS_OUTPUT', output)
        self.__output_format = output_format

        self.__log_cmd = log_cmd
        self.__on_exit = on_exit
        self.__respawn = respawn
        self.__respawn_delay = respawn_delay

        self.__process_event_args = None  # type: Optional[Dict[Text, Any]]
        self._subprocess_protocol = None  # type: Optional[Any]
        self._subprocess_transport = None
        self.__completed_future = None  # type: Optional[asyncio.Future]
        self.__shutdown_future = None  # type: Optional[asyncio.Future]
        self.__sigterm_timer = None  # type: Optional[TimerAction]
        self.__sigkill_timer = None  # type: Optional[TimerAction]
        self.__stdout_buffer = io.StringIO()
        self.__stderr_buffer = io.StringIO()

        self.__executed = False

    @classmethod
    def _parse_cmdline(
        cls,
        cmd: Text,
        parser: Parser
    ) -> List[SomeSubstitutionsType]:
        """
        Parse text apt for command line execution.

        :param: cmd a space (' ') delimited command line arguments list.
           All found `TextSubstitution` items are split and added to the
           list again as a `TextSubstitution`.
        :returns: a list of command line arguments.
        """
        result_args = []
        arg = []

        def _append_arg():
            nonlocal arg
            result_args.append(arg)
            arg = []
        for sub in parser.parse_substitution(cmd):
            if isinstance(sub, TextSubstitution):
                tokens = shlex.split(sub.text)
                if not tokens:
                    # Sting with just spaces.
                    # Appending args allow splitting two substitutions
                    # separated by a space.
                    # e.g.: `$(subst1 asd) $(subst2 bsd)` will be two separate arguments.
                    _append_arg()
                    continue
                if sub.text[0].isspace():
                    # Needed for splitting from the previous argument
                    # e.g.: `$(find-exec bsd) asd`
                    # It splits `asd` from the path of `bsd` executable.
                    if len(arg) != 0:
                        _append_arg()
                arg.append(TextSubstitution(text=tokens[0]))
                if len(tokens) > 1:
                    # Needed to split the first argument when more than one token.
                    # e.g. `$(find-pkg-prefix csd)/asd bsd`
                    # will split `$(find-pkg-prefix csd)/asd` from `bsd`.
                    _append_arg()
                    arg.append(TextSubstitution(text=tokens[-1]))
                if len(tokens) > 2:
                    # If there are more than two tokens, just add all the middle tokens to
                    # `result_args`.
                    # e.g. `$(find-pkg-prefix csd)/asd bsd dsd xsd`
                    # 'bsd' 'dsd' will be added.
                    result_args.extend([TextSubstitution(text=x)] for x in tokens[1:-1])
                if sub.text[-1].isspace():
                    # Allows splitting from next argument.
                    # e.g. `exec $(find-some-file)`
                    # Will split `exec` argument from the result of `find-some-file` substitution.
                    _append_arg()
            else:
                arg.append(sub)
        if arg:
            result_args.append(arg)
        return result_args

    @classmethod
    def parse(
        cls,
        entity: Entity,
        parser: Parser,
        ignore: Optional[List[str]] = None
    ):
        """
        Return the `ExecuteProcess` action and kwargs for constructing it.

        :param: ignore A list of arguments that should be ignored while parsing.
            Intended for code reuse in derived classes (e.g.: launch_ros.actions.Node).
        """
        _, kwargs = super().parse(entity, parser)

        if ignore is None:
            ignore = []

        if 'cmd' not in ignore:
            kwargs['cmd'] = cls._parse_cmdline(entity.get_attr('cmd'), parser)

        if 'cwd' not in ignore:
            cwd = entity.get_attr('cwd', optional=True)
            if cwd is not None:
                kwargs['cwd'] = parser.parse_substitution(cwd)

        if 'name' not in ignore:
            name = entity.get_attr('name', optional=True)
            if name is not None:
                kwargs['name'] = parser.parse_substitution(name)

        if 'prefix' not in ignore:
            prefix = entity.get_attr('launch-prefix', optional=True)
            if prefix is not None:
                kwargs['prefix'] = parser.parse_substitution(prefix)

        if 'output' not in ignore:
            output = entity.get_attr('output', optional=True)
            if output is not None:
                kwargs['output'] = parser.escape_characters(output)

        if 'respawn' not in ignore:
            respawn = entity.get_attr('respawn', data_type=bool, optional=True)
            if respawn is not None:
                kwargs['respawn'] = respawn

        if 'respawn_delay' not in ignore:
            respawn_delay = entity.get_attr('respawn_delay', data_type=float, optional=True)
            if respawn_delay is not None:
                if respawn_delay < 0.0:
                    raise ValueError(
                        'Attribute respawn_delay of Entity node expected to be '
                        'a non-negative value but got `{}`'.format(respawn_delay)
                    )
                kwargs['respawn_delay'] = respawn_delay

        if 'shell' not in ignore:
            shell = entity.get_attr('shell', data_type=bool, optional=True)
            if shell is not None:
                kwargs['shell'] = shell

        if 'additional_env' not in ignore:
            # Conditions won't be allowed in the `env` tag.
            # If that feature is needed, `set_enviroment_variable` and
            # `unset_enviroment_variable` actions should be used.
            env = entity.get_attr('env', data_type=List[Entity], optional=True)
            if env is not None:
                kwargs['additional_env'] = {
                    tuple(parser.parse_substitution(e.get_attr('name'))):
                    parser.parse_substitution(e.get_attr('value')) for e in env
                }
                for e in env:
                    e.assert_entity_completely_parsed()
        return cls, kwargs

    @property
    def output(self):
        """Getter for output."""
        return self.__output

    @property
    def process_details(self):
        """Getter for the process details, e.g. name, pid, cmd, etc., or None if not started."""
        return self.__process_event_args

    def get_sub_entities(self):
        if isinstance(self.__on_exit, list):
            return self.__on_exit
        return []

    def _shutdown_process(self, context, *, send_sigint):
        if self.__shutdown_future is None or self.__shutdown_future.done():
            # Execution not started or already done, nothing to do.
            return None
        self.__shutdown_future.set_result(None)
        if self.__completed_future is None:
            # Execution not started so nothing to do, but self.__shutdown_future should prevent
            # execution from starting in the future.
            return None
        if self.__completed_future.done():
            # If already done, then nothing to do.
            return None
        # Otherwise process is still running, start the shutdown procedures.
        context.extend_locals({'process_name': self.process_details['name']})
        actions_to_return = self.__get_shutdown_timer_actions()
        if send_sigint:
            actions_to_return.append(self.__get_sigint_event())
        return actions_to_return

    def __on_shutdown_process_event(
        self,
        context: LaunchContext
    ) -> Optional[LaunchDescription]:
        typed_event = cast(ShutdownProcess, context.locals.event)
        if not typed_event.process_matcher(self):
            # this event whas not intended for this process
            return None
        return self._shutdown_process(context, send_sigint=True)

    def __on_signal_process_event(
        self,
        context: LaunchContext
    ) -> Optional[LaunchDescription]:
        typed_event = cast(SignalProcess, context.locals.event)
        if not typed_event.process_matcher(self):
            # this event whas not intended for this process
            return None
        if self.process_details is None:
            raise RuntimeError('Signal event received before execution.')
        if self._subprocess_transport is None:
            raise RuntimeError('Signal event received before subprocess transport available.')
        if self._subprocess_protocol.complete.done():
            # the process is done or is cleaning up, no need to signal
            self.__logger.debug(
                "signal '{}' not set to '{}' because it is already closing".format(
                    typed_event.signal_name, self.process_details['name']),
            )
            return None
        if platform.system() == 'Windows' and typed_event.signal_name == 'SIGINT':
            # TODO(wjwwood): remove this when/if SIGINT is fixed on Windows
            self.__logger.warning(
                "'SIGINT' sent to process[{}] not supported on Windows, escalating to 'SIGTERM'"
                .format(self.process_details['name']),
            )
            typed_event = SignalProcess(
                signal_number=signal.SIGTERM,
                process_matcher=lambda process: True)
        self.__logger.info("sending signal '{}' to process[{}]".format(
            typed_event.signal_name, self.process_details['name']
        ))
        try:
            if typed_event.signal_name == 'SIGKILL':
                self._subprocess_transport.kill()  # works on both Windows and POSIX
                return None
            self._subprocess_transport.send_signal(typed_event.signal)
            return None
        except ProcessLookupError:
            self.__logger.debug(
                "signal '{}' not sent to '{}' because it has closed already".format(
                    typed_event.signal_name, self.process_details['name']
                )
            )

    def __on_process_stdin(
        self,
        event: ProcessIO
    ) -> Optional[SomeActionsType]:
        self.__logger.warning(
            "in ExecuteProcess('{}').__on_process_stdin_event()".format(id(self)),
        )
        cast(ProcessStdin, event)
        return None

    def __on_process_stdout(
        self, event: ProcessIO
    ) -> Optional[SomeActionsType]:
        to_write = event.text.decode(errors='replace')
        if self.__stdout_buffer.closed:
            # __stdout_buffer was probably closed by __flush_buffers on shutdown.  Output without
            # buffering.
            self.__stdout_logger.info(
                self.__output_format.format(line=to_write, this=self)
            )
        else:
            self.__stdout_buffer.write(to_write)
            self.__stdout_buffer.seek(0)
            last_line = None
            for line in self.__stdout_buffer:
                if line.endswith(os.linesep):
                    self.__stdout_logger.info(
                        self.__output_format.format(line=line[:-len(os.linesep)], this=self)
                    )
                else:
                    last_line = line
                    break
            self.__stdout_buffer.seek(0)
            self.__stdout_buffer.truncate(0)
            if last_line is not None:
                self.__stdout_buffer.write(last_line)

    def __on_process_stderr(
        self, event: ProcessIO
    ) -> Optional[SomeActionsType]:
        to_write = event.text.decode(errors='replace')
        if self.__stderr_buffer.closed:
            # __stderr buffer was probably closed by __flush_buffers on shutdown.  Output without
            # buffering.
            self.__stderr_logger.info(
                self.__output_format.format(line=to_write, this=self)
            )
        else:
            self.__stderr_buffer.write(to_write)
            self.__stderr_buffer.seek(0)
            last_line = None
            for line in self.__stderr_buffer:
                if line.endswith(os.linesep):
                    self.__stderr_logger.info(
                        self.__output_format.format(line=line[:-len(os.linesep)], this=self)
                    )
                else:
                    last_line = line
                    break
            self.__stderr_buffer.seek(0)
            self.__stderr_buffer.truncate(0)
            if last_line is not None:
                self.__stderr_buffer.write(last_line)

    def __flush_buffers(self, event, context):
        line = self.__stdout_buffer.getvalue()
        if line != '':
            self.__stdout_logger.info(
                self.__output_format.format(line=line, this=self)
            )

        line = self.__stderr_buffer.getvalue()
        if line != '':
            self.__stderr_logger.info(
                self.__output_format.format(line=line, this=self)
            )

        # the respawned process needs to reuse these StringIO resources,
        # close them only after receiving the shutdown
        if self.__shutdown_future is None or self.__shutdown_future.done():
            self.__stdout_buffer.close()
            self.__stderr_buffer.close()
        else:
            self.__stdout_buffer.seek(0)
            self.__stdout_buffer.truncate(0)
            self.__stderr_buffer.seek(0)
            self.__stderr_buffer.truncate(0)

    def __on_shutdown(self, event: Event, context: LaunchContext) -> Optional[SomeActionsType]:
        due_to_sigint = cast(Shutdown, event).due_to_sigint
        return self._shutdown_process(
            context,
            send_sigint=not due_to_sigint or context.noninteractive,
        )

    def __get_shutdown_timer_actions(self) -> List[Action]:
        base_msg = \
            "process[{}] failed to terminate '{}' seconds after receiving '{}', escalating to '{}'"

        def printer(context, msg, timeout_substitutions):
            self.__logger.error(msg.format(
                context.locals.process_name,
                perform_substitutions(context, timeout_substitutions),
            ))

        sigterm_timeout = self.__sigterm_timeout
        sigkill_timeout = [PythonExpression(
            ('float(', *self.__sigterm_timeout, ') + float(', *self.__sigkill_timeout, ')')
        )]
        # Setup a timer to send us a SIGTERM if we don't shutdown quickly.
        self.__sigterm_timer = TimerAction(
            period=sigterm_timeout,
            actions=[
                OpaqueFunction(
                    function=printer,
                    args=(base_msg.format('{}', '{}', 'SIGINT', 'SIGTERM'), sigterm_timeout)
                ),
                EmitEvent(event=SignalProcess(
                    signal_number=signal.SIGTERM,
                    process_matcher=matches_action(self)
                )),
            ],
            cancel_on_shutdown=False,
        )
        # Setup a timer to send us a SIGKILL if we don't shutdown after SIGTERM.
        self.__sigkill_timer = TimerAction(
            period=sigkill_timeout,
            actions=[
                OpaqueFunction(
                    function=printer,
                    args=(base_msg.format('{}', '{}', 'SIGTERM', 'SIGKILL'), sigkill_timeout)
                ),
                EmitEvent(event=SignalProcess(
                    signal_number='SIGKILL',
                    process_matcher=matches_action(self)
                ))
            ],
            cancel_on_shutdown=False,
        )
        return [
            cast(Action, self.__sigterm_timer),
            cast(Action, self.__sigkill_timer),
        ]

    def __get_sigint_event(self):
        return EmitEvent(event=SignalProcess(
            signal_number=signal.SIGINT,
            process_matcher=matches_action(self),
        ))

    def __cleanup(self):
        # Cancel any pending timers we started.
        if self.__sigterm_timer is not None:
            self.__sigterm_timer.cancel()
        if self.__sigkill_timer is not None:
            self.__sigkill_timer.cancel()
        # Close subprocess transport if any.
        if self._subprocess_transport is not None:
            self._subprocess_transport.close()
        # Signal that we're done to the launch system.
        self.__completed_future.set_result(None)

    class __ProcessProtocol(AsyncSubprocessProtocol):
        def __init__(
            self,
            action: 'ExecuteProcess',
            context: LaunchContext,
            process_event_args: Dict,
            **kwargs
        ) -> None:
            super().__init__(**kwargs)
            self.__context = context
            self.__action = action
            self.__process_event_args = process_event_args
            self.__logger = launch.logging.get_logger(process_event_args['name'])

        def connection_made(self, transport):
            self.__logger.info(
                'process started with pid [{}]'.format(transport.get_pid()),
            )
            super().connection_made(transport)
            self.__process_event_args['pid'] = transport.get_pid()
            self.__action._subprocess_transport = transport

        def on_stdout_received(self, data: bytes) -> None:
            self.__context.emit_event_sync(ProcessStdout(text=data, **self.__process_event_args))

        def on_stderr_received(self, data: bytes) -> None:
            self.__context.emit_event_sync(ProcessStderr(text=data, **self.__process_event_args))

    def __expand_substitutions(self, context):
        # expand substitutions in arguments to async_execute_process()
        cmd = [perform_substitutions(context, x) for x in self.__cmd]
        name = os.path.basename(cmd[0]) if self.__name is None \
            else perform_substitutions(context, self.__name)
        cmd = shlex.split(perform_substitutions(context, self.__prefix)) + cmd
        with _global_process_counter_lock:
            global _global_process_counter
            _global_process_counter += 1
            self.__name = '{}-{}'.format(name, _global_process_counter)
        cwd = None
        if self.__cwd is not None:
            cwd = ''.join([context.perform_substitution(x) for x in self.__cwd])
        env = None
        if self.__env is not None:
            env = {}
            for key, value in self.__env:
                env[''.join([context.perform_substitution(x) for x in key])] = \
                    ''.join([context.perform_substitution(x) for x in value])
        if self.__additional_env is not None:
            if env is None:
                env = dict(os.environ)
            for key, value in self.__additional_env:
                env[''.join([context.perform_substitution(x) for x in key])] = \
                    ''.join([context.perform_substitution(x) for x in value])
        # store packed kwargs for all ProcessEvent based events
        self.__process_event_args = {
            'action': self,
            'name': self.__name,
            'cmd': cmd,
            'cwd': cwd,
            'env': env,
            # pid is added to the dictionary in the connection_made() method of the protocol.
        }

    async def __execute_process(self, context: LaunchContext) -> None:
        process_event_args = self.__process_event_args
        if process_event_args is None:
            raise RuntimeError('process_event_args unexpectedly None')
        cmd = process_event_args['cmd']
        cwd = process_event_args['cwd']
        env = process_event_args['env']
        if self.__log_cmd:
            self.__logger.info("process details: cmd=[{}], cwd='{}', custom_env?={}".format(
                ', '.join(cmd), cwd, 'True' if env is not None else 'False'
            ))

        emulate_tty = self.__emulate_tty
        if 'emulate_tty' in context.launch_configurations:
            emulate_tty = evaluate_condition_expression(
                context,
                normalize_to_list_of_substitutions(
                    context.launch_configurations['emulate_tty']
                ),
            )

        try:
            transport, self._subprocess_protocol = await async_execute_process(
                lambda **kwargs: self.__ProcessProtocol(
                    self, context, process_event_args, **kwargs
                ),
                cmd=cmd,
                cwd=cwd,
                env=env,
                shell=self.__shell,
                emulate_tty=emulate_tty,
                stderr_to_stdout=False,
            )
        except Exception:
            self.__logger.error('exception occurred while executing process:\n{}'.format(
                traceback.format_exc()
            ))
            self.__cleanup()
            return

        pid = transport.get_pid()

        await context.emit_event(ProcessStarted(**process_event_args))

        returncode = await self._subprocess_protocol.complete
        if returncode == 0:
            self.__logger.info('process has finished cleanly [pid {}]'.format(pid))
        else:
            self.__logger.error("process has died [pid {}, exit code {}, cmd '{}'].".format(
                pid, returncode, ' '.join(cmd)
            ))
        await context.emit_event(ProcessExited(returncode=returncode, **process_event_args))
        # respawn the process if necessary
        if not context.is_shutdown and not self.__shutdown_future.done() and self.__respawn:
            if self.__respawn_delay is not None and self.__respawn_delay > 0.0:
                # wait for a timeout(`self.__respawn_delay`) to respawn the process
                # and handle shutdown event with future(`self.__shutdown_future`)
                # to make sure `ros2 launch` exit in time
                await asyncio.wait(
                    [asyncio.sleep(self.__respawn_delay), self.__shutdown_future],
                    loop=context.asyncio_loop,
                    return_when=asyncio.FIRST_COMPLETED
                )
            if not self.__shutdown_future.done():
                context.asyncio_loop.create_task(self.__execute_process(context))
                return
        self.__cleanup()

    def execute(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        """
        Execute the action.

        This does the following:
        - register an event handler for the shutdown process event
        - register an event handler for the signal process event
        - register an event handler for the stdin event
        - configures logging for the IO process event
        - create a task for the coroutine that monitors the process
        """
        if self.__executed:
            raise RuntimeError(
                f"ExecuteProcess action '{self.name}': executed more than once: {self.describe()}"
            )
        self.__executed = True

        if context.is_shutdown:
            # If shutdown starts before execution can start, don't start execution.
            return None

        event_handlers = [
            EventHandler(
                matcher=lambda event: is_a_subclass(event, ShutdownProcess),
                entities=OpaqueFunction(function=self.__on_shutdown_process_event),
            ),
            EventHandler(
                matcher=lambda event: is_a_subclass(event, SignalProcess),
                entities=OpaqueFunction(function=self.__on_signal_process_event),
            ),
            OnProcessIO(
                target_action=self,
                on_stdin=self.__on_process_stdin,
                on_stdout=self.__on_process_stdout,
                on_stderr=self.__on_process_stderr
            ),
            OnShutdown(
                on_shutdown=self.__on_shutdown,
            ),
            OnProcessExit(
                target_action=self,
                on_exit=self.__on_exit,
            ),
            OnProcessExit(
                target_action=self,
                on_exit=self.__flush_buffers,
            ),
        ]
        for event_handler in event_handlers:
            context.register_event_handler(event_handler)

        try:
            self.__completed_future = create_future(context.asyncio_loop)
            self.__shutdown_future = create_future(context.asyncio_loop)
            self.__expand_substitutions(context)
            self.__logger = launch.logging.get_logger(self.__name)
            self.__stdout_logger, self.__stderr_logger = \
                launch.logging.get_output_loggers(self.__name, self.__output)
            context.asyncio_loop.create_task(self.__execute_process(context))
        except Exception:
            for event_handler in event_handlers:
                context.unregister_event_handler(event_handler)
            raise
        return None

    def get_asyncio_future(self) -> Optional[asyncio.Future]:
        """Return an asyncio Future, used to let the launch system know when we're done."""
        return self.__completed_future

    @property
    def name(self):
        """Getter for name."""
        return self.__name

    @property
    def cmd(self):
        """Getter for cmd."""
        return self.__cmd

    @property
    def cwd(self):
        """Getter for cwd."""
        return self.__cwd

    @property
    def env(self):
        """Getter for env."""
        return self.__env

    @property
    def additional_env(self):
        """Getter for additional_env."""
        return self.__additional_env

    @property
    def shell(self):
        """Getter for shell."""
        return self.__shell

    @property
    def prefix(self):
        """Getter for prefix."""
        return self.__prefix
