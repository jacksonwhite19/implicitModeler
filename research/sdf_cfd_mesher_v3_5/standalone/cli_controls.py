"""Reuse the copied V4 public control functions without its campaign launcher."""
from pathlib import Path
import ast,copy,hashlib,json,math
source=Path(__file__).resolve().parent/'fixtures/public_cli_reference.py'
names={'surface_configuration','surface_policy_configuration','identity'}
nodes=[n for n in ast.parse(source.read_text()).body if isinstance(n,ast.FunctionDef) and n.name in names]
if {n.name for n in nodes}!=names:raise RuntimeError('CopiedV4ControlFunctionClosureChanged')
exec(compile(ast.Module(body=nodes,type_ignores=[]),str(source),'exec'))
