import io
import os
import re

import ament_index_python


def get_default_filtered_prefixes():
    return [
        b'pid', b'rc',
    ]


def get_default_filtered_patterns():
    return []


def get_rmw_output_filter(rmw_implementation, filter_type):
    supported_filter_types = ['prefixes', 'patterns']
    if filter_type not in supported_filter_types:
        raise TypeError(
            'Unsupported filter_type "{0}". Supported types: {1}'
            .format(filter_type, supported_filter_types))
    resource_name = 'rmw_output_' + filter_type
    prefix_with_resource = ament_index_python.has_resource(
        resource_name, rmw_implementation)
    if not prefix_with_resource:
        return []

    # Treat each line of the resource as an independent filter.
    rmw_output_filter, _ = ament_index_python.get_resource(resource_name, rmw_implementation)
    return [str.encode(l) for l in rmw_output_filter.splitlines()]


def get_expected_output(output_file):
    literal_file = output_file + '.txt'
    if os.path.isfile(literal_file):
        with open(literal_file, 'rb') as f:
            return f.read().splitlines()
    regex_file = output_file + '.regex'
    if os.path.isfile(regex_file):
        with open(regex_file, 'rb') as f:
            return f.read().splitlines()


def create_output_lines_filter(filtered_prefixes, filtered_patterns,
                               filtered_rmw_implementation):
    filtered_prefixes = filtered_prefixes or get_default_filtered_prefixes()
    filtered_patterns = filtered_patterns or get_default_filtered_patterns()
    if filtered_rmw_implementation:
        filtered_prefixes.extend(get_rmw_output_filter(
            filtered_rmw_implementation, 'prefixes'
        ))
        filtered_patterns.extend(get_rmw_output_filter(
            filtered_rmw_implementation, 'patterns'
        ))
    filtered_patterns = map(re.compile, filtered_patterns)

    def _filter(output):
        for line in output.splitlines():
            # Filter out stdout that comes from underlying DDS implementation
            # Note: we do not currently support matching filters across multiple stdout lines.
            if any(line.startswith(prefix) for prefix in filtered_prefixes):
                continue
            if any(pattern.match(line) for pattern in filtered_patterns):
                continue
            yield line
    return _filter


def create_output_check(output_file, filtered_prefixes, filtered_patterns,
                        filtered_rmw_implementation=None):
    filter_output_lines = create_output_lines_filter(
        filtered_prefixes, filtered_patterns, filtered_rmw_implementation
    )

    literal_file = output_file + '.txt'
    if os.path.isfile(literal_file):
        def _collate(output, addendum):
            output.extend(filter_output_lines(addendum))
            return output

        def _match(output, pattern):
            return pattern in output

        with open(literal_file, 'rb') as f:
            expected_output = f.read().splitlines()
        return [], _collate, _match, expected_output

    regex_file = output_file + '.regex'
    if os.path.isfile(regex_file):
        def _collate(output, addendum):
            output.write(b'\n'.join(
                filter_output_lines(addendum)
            ))
            return output

        def _match(output, pattern):
            return pattern.search(output.getvalue()) is not None

        with open(regex_file, 'rb') as f:
            patterns = map(re.compile, f.read().splitlines())
        return io.BytesIO(), _collate, _match, patterns

    raise RuntimeError('could not find output check file: {}'.format(output_file))
