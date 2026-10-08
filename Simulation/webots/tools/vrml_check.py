"""Offline structural checker using Cyberbotics R2025a node field definitions.

Checks field names/types, syntax and local asset paths. This is NOT a replacement
for Webots' parser, physics engine or a real launch test.
"""
from dataclasses import dataclass, field
import json
from pathlib import Path
import re

TOKEN = re.compile(r'\s+|\#[^\n]*|"(?:\\.|[^"\\])*"|[A-Za-z_]\w*|[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?|[{}\[\],]')
COUNTS = {'SFVec2f':2, 'SFVec3f':3, 'SFColor':3, 'SFRotation':4}


@dataclass
class Node:
    kind: str
    fields: dict = field(default_factory=dict)
    name: str = ''


class Parser:
    def __init__(self, path, schema):
        self.path, self.schema = Path(path), dict(schema)
        self.tokens = []
        source = self.path.read_text(encoding='utf-8')
        end = 0
        for m in TOKEN.finditer(source):
            if source[end:m.start()].strip():
                raise ValueError(f'Unrecognized text: {source[end:m.start()]!r}')
            end = m.end()
            if not m[0].isspace() and not m[0].startswith('#') and m[0] != ',':
                self.tokens.append(m[0])
        if source[end:].strip():
            raise ValueError('Trailing unrecognized text.')
        self.i, self.defs, self.nodes, self.externs, self.interface = 0, {}, [], [], {}

    def pop(self, expected=None):
        if self.i >= len(self.tokens):
            raise ValueError(f'{self.path.name}: unexpected end of file')
        value = self.tokens[self.i]
        self.i += 1
        if expected is not None and value != expected:
            raise ValueError(f'{self.path.name}: expected {expected}, got {value}')
        return value

    def peek(self):
        return self.tokens[self.i] if self.i < len(self.tokens) else None

    def value(self, kind):
        if self.peek() == 'IS':
            self.pop()
            name = self.pop()
            if self.interface.get(name) != kind:
                raise ValueError(f'IS {name}: incompatible type {kind}')
            return ('IS', name)
        if kind.startswith('MF'):
            sf = 'SF'+kind[2:]
            result = []
            if self.peek() == '[':
                self.pop('[')
                while self.peek() != ']':
                    result.append(self.value(sf))
                self.pop(']')
            else:
                result.append(self.value(sf))
            return result
        if kind == 'SFNode':
            return self.node()
        if kind == 'SFString':
            token = self.pop()
            if not token.startswith('"'):
                raise ValueError(f'Expected string, got {token}')
            return json.loads(token)
        if kind == 'SFBool':
            token = self.pop()
            if token not in ('TRUE','FALSE'):
                raise ValueError('Boolean needs TRUE/FALSE.')
            return token == 'TRUE'
        if kind in COUNTS:
            return [float(self.pop()) for _ in range(COUNTS[kind])]
        token = self.pop()
        return int(token) if kind == 'SFInt32' else float(token)

    def node(self):
        token, name = self.pop(), ''
        if token == 'NULL':
            return None
        if token == 'USE':
            return self.defs[self.pop()]
        if token == 'DEF':
            name, token = self.pop(), self.pop()
        if token not in self.schema:
            raise ValueError(f'Unknown node type {token} in {self.path.name}')
        result = Node(token, name=name)
        if name:
            if name in self.defs:
                raise ValueError(f'Duplicate DEF {name}')
            self.defs[name] = result
        self.nodes.append(result)
        self.pop('{')
        while self.peek() != '}':
            key = self.pop()
            if key not in self.schema[token]:
                raise ValueError(f'Unknown field {token}.{key}')
            if key in result.fields:
                raise ValueError(f'Duplicate field {token}.{key}')
            result.fields[key] = self.value(self.schema[token][key])
        self.pop('}')
        return result

    def parse(self):
        roots = []
        while self.peek() is not None:
            if self.peek() == 'EXTERNPROTO':
                self.pop()
                url = self.value('SFString')
                path = (self.path.parent/url).resolve()
                external = Parser(path,self.schema)
                external.parse()
                name = next(iter(external.proto_names))
                self.schema[name] = external.interface
                self.externs.append(path)
            elif self.peek() == 'PROTO':
                self.pop()
                name = self.pop()
                self.proto_names = [name]
                self.pop('[')
                while self.peek() != ']':
                    self.pop('field')
                    kind, key = self.pop(), self.pop()
                    self.interface[key] = kind
                    self.value(kind)
                self.pop(']')
                self.pop('{')
                roots.append(self.node())
                self.pop('}')
            else:
                roots.append(self.node())
        return roots

    def check_assets(self):
        for node in self.nodes:
            if node.kind in ('Mesh','ImageTexture'):
                for url in node.fields.get('url', []):
                    if '://' in url:
                        raise ValueError('Project must work without remote assets.')
                    if not (self.path.parent/url).is_file():
                        raise ValueError(f'Missing asset {url}')


def load_schema(project):
    return json.loads((Path(project)/'tests/fixtures/webots_R2025a_fields.json').read_text(encoding='utf-8'))['nodes']
