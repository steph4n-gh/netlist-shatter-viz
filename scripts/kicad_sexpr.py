"""Small KiCad token parser shared with the bundled Python 3.9 runtime."""
import re

def zone_definition(item):
    """Exact zone definition without native fill caches; rules/outline stay exact."""
    item=[v for v in item if not (isinstance(v,list) and v and v[0] in
          ('filled_polygon','fill_segments','filled_areas_thickness'))]
    return [[v[0]]+[x for x in v[1:] if x!='yes'] if isinstance(v,list) and v and v[0]=='fill' else v for v in item]

def sexpr(text, literal_strings=False):
    """Read a token tree without interpreting strings or executing code."""
    stack = []
    root = None
    tokens = r'\s+|"[^"]*"|[()]|[^\s()"]+' if literal_strings else r'\s+|"(?:\\.|[^"\\])*"|[()]|[^\s()"]+'
    for match in re.finditer(tokens, text):
        token = match.group()
        if token.isspace():
            continue
        if token == '(':
            node = []
            if stack:
                stack[-1].append(node)
            elif root is not None:
                raise ValueError('Multiple s-expression roots')
            else:
                root = node
            stack.append(node)
        elif token == ')':
            if not stack:
                raise ValueError('Unbalanced s-expression')
            stack.pop()
        elif stack:
            stack[-1].append(token)
        else:
            raise ValueError('Token outside s-expression')
    if stack or root is None:
        raise ValueError('Incomplete s-expression')
    return root
