"""The single public V4 mesher command. No case-specific execution entry."""
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent
sys.path[:0]=[str(HERE/'src'),str(HERE/'tools')]
sys.dont_write_bytecode=True
from standalone_tool import main
if __name__=='__main__':raise SystemExit(main())
