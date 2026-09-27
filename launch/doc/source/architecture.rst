Architecture of `launch`
========================

`launch` is designed to provide core features like describing actions (e.g. executing a process or including another launch description), generating events, introspecting launch descriptions, and executing launch descriptions.
At the same time, it provides extension points so that the set of things that these core features can operate on, or integrate with, can be expanded with additional packages.

Launch Entities and Launch Descriptions
---------------------------------------

The main object in `launch` is the :class:`launch.LaunchDescriptionEntity`, from which other entities that are "launched" inherit.
This class, or more specifically classes derived from this class, are responsible for capturing the system architect's (a.k.a. the user's) intent for how the system should be launched, as well as how `launch` itself should react to asynchronous events in the system during launch.
A launch description entity has its :meth:`launch.LaunchDescriptionEntity.visit` method called during "launching", and has any of the "describe" methods called during "introspection".
It may also provide a :class:`asyncio.Future` with the :meth:`launch.LaunchDescriptionEntity.get_asyncio_future` method, if it has on-going asynchronous activity after returning from visit.

When visited, entities may yield additional entities to be visited, and this pattern is used from the "root" of the launch, where a special entity called :class:`launch.LaunchDescription` is provided to start the launch process.
These entities are visited recursively and depth-first.

The :class:`launch.LaunchDescription` class encapsulates the intent of the user as a list of discrete :class:`launch.Action`'s, which are also derived from :class:`launch.LaunchDescriptionEntity`.
As "launch description entities" themselves, these "actions" can either be introspected for analysis without performing the side effects, or the actions can be executed, usually in response to an event in the launch system.

Additionally, launch descriptions, and the actions that they contain, can have references to :class:`launch.Substitution`'s within them.
These substitutions are things that can be evaluated during launch and can be used to do various things like: get a launch configuration, get an environment variable, or evaluate arbitrary Python expressions.

Launch descriptions, and the actions contained therein, can either be introspected directly or launched by a :class:`launch.LaunchService`.
A launch service is a long running activity that coordinates runtime execution and entity visitation.

Actions
-------

The aforementioned actions allow the user to express various intentions, and the set of available actions to the user can also be extended by other packages, allowing for domain specific actions.

Actions can have direct side effects (e.g. run a process or set a configuration variable) and as well they can yield additional actions.
The latter can be used to create "syntactic sugar" actions which simply yield more verbose actions.

Actions may also have arguments, which can affect the behavior of the actions.
These arguments are where :class:`launch.Substitution`'s can be used to provide more flexibility when describing reusable launch descriptions.

Basic Actions
^^^^^^^^^^^^^

`launch` provides the foundational actions on which other more sophisticated actions may be built.
This is a non-exhaustive list of actions that `launch` may provide:

- :class:`launch.actions.IncludeLaunchDescription`

  - This action will include another launch description as if it had been copy-pasted to the location of the include action.

- :class:`launch.actions.SetLaunchConfiguration`

  - This action will set a :class:`launch.LaunchConfiguration` to a specified value, creating it if it doesn't already exist.
  - These launch configurations can be accessed by any action via a substitution, but are scoped by default.

- :class:`launch.actions.DeclareLaunchArgument`

  - This action will declare a launch description argument, which can have a name, default value, and documentation.
  - The argument will be exposed via a command line option for a root launch description, or as action configurations to the include launch description action for the included launch description.

- :class:`launch.actions.SetEnvironmentVariable`

  - This action will set an environment variable by name.

- :class:`launch.actions.AppendEnvironmentVariable`

  - This action will set an environment variable by name if it does not exist, otherwise it appends to the existing value using a platform-specific separator.
  - There is also an option to prepend instead of appending and to provide a custom separator.

- :class:`launch.actions.GroupAction`

  - This action will yield other actions, but can be associated with conditionals (allowing you to use the conditional on the group action rather than on each sub-action individually) and can optionally scope the launch configurations.

- :class:`launch.actions.TimerAction`

  - This action will yield other actions after a period of time has passed without being canceled.

- :class:`launch.actions.ExecuteProcess`

  - This action will execute a process given its path and arguments, and optionally other things like working directory or environment variables.

- :class:`launch.actions.RegisterEventHandler`

  - This action will register an :class:`launch.EventHandler` class, which takes a user defined lambda to handle some event.
  - It could be any event, a subset of events, or one specific event.

- :class:`launch.actions.UnregisterEventHandler`

  - This action will remove a previously registered event.

- :class:`launch.actions.EmitEvent`

  - This action will emit an :class:`launch.Event` based class, causing all registered event handlers that match it to be called.

- :class:`launch.actions.LogInfo`:

  - This action will log a user defined message to the logger, other variants (e.g. ``LogWarn``) could also exist.

- :class:`launch.actions.RaiseError`

  - This action will stop execution of the launch system and provide a user defined error message.

More actions can always be defined via extension, and there may even be additional actions defined by `launch` itself, but they are more situational and would likely be built on top of the above actions anyways.

Base Action
^^^^^^^^^^^

All actions need to inherit from the :class:`launch.Action` base class, so that some common interface is available to the launch system when interacting with actions defined by external packages.
Since the base action class is a first class element in a launch description it also inherits from :class:`launch.LaunchDescriptionEntity`, which is the polymorphic type used when iterating over the elements in a launch description.

Also, the base action has a few features common to all actions, such as some introspection utilities, and the ability to be associated with a single :class:`launch.Condition`, like the :class:`launch.IfCondition` class or the :class:`launch.UnlessCondition` class.

The action configurations are supplied when the user uses an action and can be used to pass "arguments" to the action in order to influence its behavior, e.g. this is how you would pass the path to the executable in the execute process action.

If an action is associated with a condition, that condition is evaluated to determine if the action is executed or not.
Even if the associated action evaluates to false the action will be available for introspection.

Events and Event Handlers
-------------------------

A :class:`launch.Event` represents an occurrence to which the launch system may react.
It may carry information about that occurrence for use by event handlers.
Events are the only entry point by which a :class:`launch.LaunchService` begins runtime visitation of entities.
Entities do not execute themselves, and the launch service does not visit an entity merely because it has been constructed or added to a launch description.

Emitting an event places it in the launch context's event queue.
Emission does not immediately invoke event handlers.
The launch service takes events from the queue one at a time and checks each event against all registered event handlers.
Every matching handler is invoked and may perform side effects or return one or more entities.
The event itself does not return entities; its matching handlers do.

Entities returned by a handler form the roots of entity-visitation chains.
The launch service visits each root and all entities returned by it recursively and depth-first.
A separate event is not required between a parent entity and each of its returned entities.

While being visited, an entity may emit another event directly or arrange for asynchronous activity to emit one later.
The emitted event is queued for later processing, so the current depth-first entity visitation completes before the new event is handled.
This creates the central execution cycle:

.. code-block:: text

   event
       |
       v
   matching event handlers
       |
       v
   returned entity trees
       |
       v
   depth-first visitation
       |
       `-- emitted events -> event queue

Event handlers are represented by the :class:`launch.EventHandler` base class.
They define two main methods: :meth:`launch.EventHandler.matches` and :meth:`launch.EventHandler.handle`.
The ``matches()`` method receives the event and returns ``True`` if the handler should handle it.
The ``handle()`` method receives the event and launch context, and may perform side effects or return entities for the launch service to visit.
Event handlers do not inherit from :class:`launch.LaunchDescriptionEntity` and are not visited through the entity visitation protocol.

Several actions connect entity visitation to this event system:

- :class:`launch.actions.RegisterEventHandler` registers an event handler.
- :class:`launch.actions.UnregisterEventHandler` removes a previously registered event handler.
- :class:`launch.actions.EmitEvent` emits an event when the action is visited.

Substitutions
-------------

A substitution is something that cannot, or should not, be evaluated until it's time to execute the launch description that they are used in.
There are many possible variations of a substitution, but here are some of the core ones implemented by `launch` (all of which inherit from :class:`launch.Substitution`):

- :class:`launch.substitutions.Text`

  - This substitution simply returns the given string when evaluated.
  - It is usually used to normalize literals by wrapping them into substitutions in the launch description so they can be concatenated with other substitutions (see :func:`launch.utilities.normalize_to_list_of_substitutions`).
  - An iterable of substitutions is concatenated into a single string (see :func:`launch.utilities.perform_substitutions`).

- :class:`launch.substitutions.PythonExpression`

  - This substitution will evaluate a python expression and get the result as a string.
  - You may pass a list of Python modules to the constructor to allow the use of those modules in the evaluated expression.

- :class:`launch.substitutions.StringStripSubstitution`

  - This substitution removes leading and trailing whitespace from the result of one or more substitutions.
  - It can remove a newline terminator from command output before composition, for example ``$(string-strip $(command 'hostname'))``.

- :class:`launch.substitutions.LaunchConfiguration`

  - This substitution gets a launch configuration value, as a string, by name.

- :class:`launch.substitutions.IfElseSubstitution`

  - This substitution takes a substitution, and if it evaluates to true, then the result is the if_value, else the result is the else_value.

- :class:`launch.substitutions.LocalSubstitution`

  - This substitution gets a "local" variable out of the context. This is a mechanism that allows a "parent" action to pass information to sub actions.
  - As an example, consider this pseudo code example ``OnShutdown(actions=LogInfo(msg=["shutdown due to: ", LocalSubstitution(expression='event.reason')]))``, which assumes that ``OnShutdown`` will put the shutdown event in the locals before ``LogInfo`` is visited.

- :class:`launch.substitutions.EnvironmentVariable`

  - This substitution gets an environment variable value, as a string, by name.

- :class:`launch.substitutions.FindExecutable`

  - This substitution locates the full path to an executable on the PATH if it exists.

The base substitution class provides some common introspection interfaces (which the specific derived substitutions may influence).

The Launch Service
------------------

The launch service is responsible for processing emitted events, dispatching them to event handlers, and executing actions as needed.
The launch service offers three main services:

- include a launch description

  - can be called from any thread

- run event loop
- shutdown

  - cancels any running actions and event handlers
  - then breaks the event loop if running
  - can be called from any thread

Starting Execution
^^^^^^^^^^^^^^^^^^

The entry point for executing a description is the :class:`launch.LaunchService` API:

.. code-block:: python

   import launch

   launch_service = launch.LaunchService()
   launch_service.include_launch_description(launch_description)
   return_code = launch_service.run()

The launch service operates on an already constructed :class:`launch.LaunchDescription`.
Obtaining that description, whether programmatically, from a file, or through another frontend, is the responsibility of the embedding application.

Calling :meth:`launch.LaunchService.include_launch_description` queues a :class:`launch.events.IncludeLaunchDescription` event.
A built-in event handler turns that event into the root launch description entity, which is then visited through the normal execution model.
:meth:`launch.LaunchService.run` and :meth:`launch.LaunchService.run_async` process queued work; they do not create or load a root launch description automatically.

There are two similarly named include mechanisms:

- :meth:`launch.LaunchService.include_launch_description` introduces a root launch description into a launch service by emitting a :class:`launch.events.IncludeLaunchDescription` event.
- :class:`launch.actions.IncludeLaunchDescription` is an entity within a launch description. When visited, it obtains another launch description from its source and yields that description for visitation.

.. _launch-service-execution-model:

Execution Model
^^^^^^^^^^^^^^^

The launch service coordinates event processing and entity visitation as follows:

#. Events are placed in the launch context's event queue.
#. The launch service takes one event from the queue and checks it against the registered event handlers.
#. Every matching event handler is invoked and may return launch description entities.
#. Returned entities are visited synchronously and depth-first.
#. Visiting an entity may perform an immediate side effect, return sub-entities for immediate visitation, start asynchronous work represented by a future, or emit another event.
#. The launch service tracks asynchronous work and waits for another event or for tracked work to complete.
#. When no events or asynchronous work remain, the default behavior is to emit a shutdown event. The service exits after the shutdown event and any work it produces has completed.

The flow can be summarized as:

.. code-block:: text

   event queue
       |
       v
   event
       |
       v
   matching event handlers
       |
       v
   returned entities
       |
       v
   depth-first visitation
       |-- returned sub-entities -> visit immediately
       |-- asynchronous future   -> track until complete
       `-- emitted event         -> queue for later

Events emitted while handling an event or visiting its entities are queued.
They are not processed recursively; the current depth-first entity traversal completes first.

Embedding and Dynamic Control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A typical embedding application would:

- create or obtain a launch description
- create a launch service
- include the launch description in the launch service
- run the launch service
- call shutdown when external circumstances require it

An application may also host an external control interface in another thread.
Such an interface can dynamically include launch descriptions or request shutdown while the launch service is running.
A launch description can itself contain actions that register event handlers, emit events, run processes, and perform other work, so asynchronous inclusion provides a general mechanism for dynamically extending a running launch system.

Extension Points
----------------

In order to allow customization of how `launch` is used in specific domains, extension of the core categories of features is provided.
External Python packages, through extension points, may add:

- new actions

  - must directly or indirectly inherit from :class:`launch.Action`

- new events

  - must directly or indirectly inherit from :class:`launch.Event`

- new substitutions

  - must directly or indirectly inherit from :class:`launch.Substitution`

- kinds of entities in the launch description

  - must directly or indirectly inherit from :class:`launch.LaunchDescriptionEntity`

In the future, more traditional extensions (like with ``setuptools``' ``entry_point`` feature) may be available via the launch service, e.g. the ability to include some extra entities and event handlers before the launch description is included.
