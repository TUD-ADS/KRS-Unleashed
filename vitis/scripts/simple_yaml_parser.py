import os


class SimpleYAMLParser:
    """
    A simple YAML parser supporting nested dictionaries, lists of objects, and multi-line strings.
    Only uses Python's standard library.
    """

    def __init__(self):
        self.indent_stack = [0]

    def parse_value(self, value):
        value = value.strip()
        if value == '':
            return None
        if value.lower() in ('true', 'yes', 'on'):
            return True
        if value.lower() in ('false', 'no', 'off'):
            return False
        if value.lower() in ('null', 'none', '~'):
            return None
        try:
            if '.' in value:
                return float(value)
            return int(value)
        except ValueError:
            pass
        if (value.startswith('"') and value.endswith('"')) or (value.startswith("'") and value.endswith("'")):
            return value[1:-1]
        return value

    def parse(self, text):
        lines = text.split('\n')
        result = {}
        context_stack = [result]
        context_types = ['dict']
        last_key = None
        multiline_mode = None
        multiline_key = None
        multiline_indent = 0
        multiline_content = []

        i = 0
        while i < len(lines):
            line = lines[i]
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                i += 1
                continue

            indent = len(line) - len(line.lstrip())

            # Handle multi-line string mode
            if multiline_mode:
                if indent > multiline_indent and line.strip():
                    multiline_content.append(line[multiline_indent:])
                    i += 1
                    continue
                else:
                    # End of multi-line
                    if multiline_mode == 'literal':
                        value = '\n'.join(multiline_content)
                    else:
                        value = ' '.join(multiline_content)
                    context_stack[-1][multiline_key] = value
                    multiline_mode = None
                    multiline_key = None
                    multiline_content = []
                    continue  # reprocess this line

            # Indentation management
            while indent < self.indent_stack[-1]:
                self.indent_stack.pop()
                context_stack.pop()
                context_types.pop()
            if indent > self.indent_stack[-1]:
                self.indent_stack.append(indent)
                # The context_stack is already updated when a new dict/list is created

            # List item
            if stripped.startswith('- '):
                value = stripped[2:].strip()
                # If current context is not a list, create one
                if not isinstance(context_stack[-1], list):
                    # Attach new list to the last key in the parent dict
                    parent = context_stack[-2]
                    parent[last_key] = []
                    context_stack[-1] = parent[last_key]
                    context_types[-1] = 'list'
                # List item can be a dict or a value
                if ':' in value:
                    key, val = value.split(':', 1)
                    item = {key.strip(): self.parse_value(val.strip())}
                    context_stack[-1].append(item)
                    # Check if next line is indented (nested dict)
                    if i + 1 < len(lines):
                        next_line = lines[i + 1]
                        next_indent = len(next_line) - len(next_line.lstrip())
                        if next_indent > indent:
                            context_stack.append(item)
                            context_types.append('dict')
                            self.indent_stack.append(next_indent)
                            last_key = key.strip()
                else:
                    context_stack[-1].append(self.parse_value(value))
            # Dict key-value
            elif ':' in stripped:
                key, val = stripped.split(':', 1)
                key = key.strip()
                val = val.strip()
                last_key = key
                if val in ('|', '>'):
                    multiline_mode = 'literal' if val == '|' else 'folded'
                    multiline_key = key
                    multiline_indent = indent + 2
                    multiline_content = []
                elif val == '':
                    # Nested dict or list to follow
                    context_stack[-1][key] = {}
                    context_stack.append(context_stack[-1][key])
                    context_types.append('dict')
                    self.indent_stack.append(indent + 2)
                else:
                    context_stack[-1][key] = self.parse_value(val)
            else:
                # Standalone value (rare in YAML, can be extended)
                pass
            i += 1
        return result

    def parse_file(self, file_path):
        with open(file_path, 'r', encoding='utf-8') as f:
            return self.parse(f.read())
