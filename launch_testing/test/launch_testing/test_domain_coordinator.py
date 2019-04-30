# Copyright 2019 Apex.AI, Inc.
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

import unittest

import launch_testing.domain_coordinator
from launch_testing.domain_coordinator import get_coordinated_domain_id


class TestUniqueness(unittest.TestCase):

    def test_quickly(self):
        """
        Quick test with false negatives, but simple and easy to understand.

        See that we generate unique domains.  Will not necessarily find problems because domains
        are selected randomly.  We're only asking for 10 domains out of 100 so most of the time
        we'll probably get lucky.
        """
        domains = [get_coordinated_domain_id() for _ in range(10)]

        domain_ids = [str(domain) for domain in domains]

        self.assertEqual(
            sorted(domain_ids),
            sorted(set(domain_ids))  # 'set' will remove duplicates
        )

    def test_with_forced_collision(self):

        domain = launch_testing.domain_coordinator.get_coordinated_domain_id(
            selector=lambda: 42  # Force it to select '42' as the domain every time it tries
        )
        self.assertEqual('42', str(domain))

        # Now that we've already got domain 42 reserved, this call should fail:
        with self.assertRaises(Exception) as cm:
            launch_testing.domain_coordinator.get_coordinated_domain_id(
                selector=lambda: 42
            )

        self.assertIn('Failed to get a unique domain ID', str(cm.exception))

    def test_known_order(self):

        class sequence_gen:

            def __init__(self):
                self._sequence = 1

            def __call__(self):
                try:
                    return self._sequence
                finally:
                    self._sequence += 1

        domains = [get_coordinated_domain_id(selector=sequence_gen()) for _ in range(10)]

        self.assertEqual(
            ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10'],
            [str(domain) for domain in domains]
        )


class TestSelector(unittest.TestCase):

    def test_selector_values_between_1_and_100(self):
        selector = launch_testing.domain_coordinator._default_selector()

        for n in range(200):
            val = selector()
            self.assertGreaterEqual(val, 1)
            self.assertLessEqual(val, 100)

    def test_selector_values_are_unique(self):
        selector = launch_testing.domain_coordinator._default_selector()

        # The default sequencer should produce 100 unique values before it starts to repeat.
        seen_values = [selector() for _ in range(100)]

        self.assertEqual(
            sorted(seen_values),
            [n + 1 for n in range(100)]
        )
