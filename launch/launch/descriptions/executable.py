"""Module for a description of an Executable."""

import os
import shlex
import threading
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Tuple

from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch.launch_context import LaunchContext
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions

_global_process_counter_lock = threading.Lock()
_global_process_counter = 0  # in Python3, this number is unbounded (no rollover)

class Executable:
    """Describes an executable (typically a single process) which may be run by the launch system."""

    def __init__(
        self, *,
        cmd: Iterable[SomeSubstitutionsType],
        name: Optional[SomeSubstitutionsType] = None,
        cwd: Optional[SomeSubstitutionsType] = None,
        env: Optional[Dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
        additional_env: Optional[Dict[SomeSubstitutionsType, SomeSubstitutionsType]] = None,
    ) -> None:
        """
        Initialize an Executable description.

        :param cmd: A list where the first item is the executable and the rest are
            arguments to the executable, each item may be a string or a list of strings
            and Substitutions to be resolved at runtime
        :param name: The label used to represent the process, as a string or a Substitution
            to be resolved at runtime, defaults to the basename of the executable
        :param cwd: The directory in which to run the executable
        :param env: Dictionary of environment variables to be used, starting from a clean
            environment. If None, the current environment is used.
        :param additional_env: Dictionary of environment variables to be added. If env was
            None, they are added to the current environment. If not, env is updated with
            additional_env.
        """
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
    def process_details(self):
        """Getter for the substituted executable details, e.g. cmd, cwd, env, or None if substitutions have not been performed."""
        return self.__process_event_args

    def apply_context(self, context: LaunchContext):
        """
        Prepares an executable description for execution in a given environment.

        This does the following:
        - performs substitutions on various properties
        """
        self.__expand_substitutions(context)
        process_event_args = self.__process_event_args
        if process_event_args is None:
            raise RuntimeError('process_event_args unexpectedly None')

    def __expand_substitutions(self, context):
        # expand substitutions in arguments to async_execute_process()
        cmd = [perform_substitutions(context, x) for x in self.__cmd]
        name = os.path.basename(cmd[0]) if self.__name is None \
            else perform_substitutions(context, self.__name)
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
            'description': self,
            'name': self.__name,
            'cmd': cmd,
            'cwd': cwd,
            'env': env,
            # pid is added to the dictionary in the connection_made() method of the protocol.
        }


