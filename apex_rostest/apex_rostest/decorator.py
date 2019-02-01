# Copyright 2019 Apex.AI, Inc.
# All rights reserved.


def post_shutdown_test():
    """Decorate tests that are meant to run after node shutdown."""
    def decorator(test_item):
        if not isinstance(test_item, type):
            raise TypeError("postcondition_test should decorate test classes")
        test_item.__post_shutdown_test__ = True
        return test_item

    return decorator
