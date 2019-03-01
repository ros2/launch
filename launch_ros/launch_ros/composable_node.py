# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Module for a description of a ComposableNode."""

from typing import Dict
from typing import Iterable
from typing import Optional
from typing import Tuple
from typing import Union

from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
# from launch.utilities import ensure_argument_type
from launch.utilities import normalize_to_list_of_substitutions
from launch_ros.remap_rule_type import RemapRules
from launch_ros.remap_rule_type import SomeRemapRules
from launch_ros.utilities import normalize_remap_rules


class ComposableNode:
    """Describes a ROS node that can be loaded into a container with other nodes."""

    def __init__(
        self, *,
        package: SomeSubstitutionsType,
        node_plugin: SomeSubstitutionsType,
        node_name: Optional[SomeSubstitutionsType] = None,
        node_namespace: Optional[SomeSubstitutionsType] = None,
        parameters:
            Optional[
                Iterable[
                    Union[SomeSubstitutionsType,
                          Dict[SomeSubstitutionsType, SomeSubstitutionsType]]]] = None,
        remappings: Optional[SomeRemapRules] = None,
        extra_arguments: Optional[Iterable[
            Tuple[SomeSubstitutionsType, SomeSubstitutionsType]]] = None,
    ) -> None:
        """
        Initialize a ComposableNode description.

        :param package: Name of the ROS package the node plugin lives in
        :param node_plugin: Name of the plugin to be loaded
        :param node_name: Name the node should have
        :param node_namespace: Namespace the node should create topics/services/etc in
        :param parameters: List of either paths to yaml files or dictionaries of parameters
        :param remappings: List of from/to pairs for remapping names
        :pram extra_arguments: Container specific arguments to be passed to the loaded node
        """
        self.__package = ()
        self.__node_plugin = ()
        self.__node_name = ()
        self.__node_namespace = ()
        self.__parameters = ()
        self.__remappings = ()
        self.__extra_arguments = ()

        if package:
            self.__package = tuple(normalize_to_list_of_substitutions(package))

        if node_plugin:
            self.__node_plugin = tuple(normalize_to_list_of_substitutions(node_plugin))

        if node_name:
            self.__node_name = tuple(normalize_to_list_of_substitutions(node_name))

        if node_namespace:
            self.__node_namespace = tuple(normalize_to_list_of_substitutions(node_namespace))

        if parameters:
            # TODO(sloretz) Need to resolve this to list of paths to parameter files
            # or a dictionary whose key is a list of substitutions and value
            # is some allowed parameter type
            raise NotImplementedError()

        if remappings:
            self.__remappings = tuple(normalize_remap_rules(remappings))

        if extra_arguments:
            self.__extra_arguments = tuple(normalize_to_list_of_substitutions(package))

    @property
    def package(self) -> Tuple[Substitution, ...]:
        """Get substitutions that can be performed to reveal the package name."""
        return self.__package

    @property
    def node_name(self) -> Tuple[Substitution, ...]:
        """Get substitutions that can be performed to reveal the node name."""
        return self.__node_name

    @property
    def node_namespace(self) -> Tuple[Substitution, ...]:
        return self.__node_namespace

    @property
    def parameters(self) -> Tuple[Substitution, ...]:
        return self.__parameters

    @property
    def remappings(self) -> RemapRules:
        return self.__remappings

    @property
    def extra_arguments(self) -> Tuple[Substitution, ...]:
        return self.__extra_arguments
