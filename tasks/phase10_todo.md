# Phase 10: Project Tree Panel — Task Plan

## Parts from spec (scratchpad.md)
- Part 1: TreeNode data model in `src/ui/project_tree.rs`
- Part 2: Tree UI with egui CollapsingHeader, icons, hierarchy
- Part 3: Click to highlight in viewport (simplified — track selection only)
- Part 4: Click to jump to script definition (text search)
- Part 5: Right-click to rename
- Part 6: Tree refresh after each eval, preserve collapse state
- Part 7: Empty state placeholder

## Steps

### Step 1 — Create `src/ui/project_tree.rs` [>]
- Define TreeNode enum, TreeNodeId, ProjectTree struct
- Implement build_tree() from available data (mass_points, fea_setup, profiles, splines)
- Implement show_project_tree() egui renderer

### Step 2 — Wire into App struct
- Add fields: project_tree, tree_collapse, selected_tree_node, pending_cursor_offset
- Register module in src/ui/mod.rs
- Call rebuild_project_tree() after execute_script()

### Step 3 — Add panel to layout
- Add SidePanel::left("project_tree") before existing code_panel
- Set 220px default width with resize handle
- Show tree content or empty state

### Step 4 — Selection + script jump
- On node click, store selected_tree_node
- Search for name in script_text to find line
- Set pending_cursor_offset, scroll editor on next frame

### Step 5 — Right-click rename
- On right-click, open small popup with text input
- On confirm, find-replace in script_text
- Show confirmation if name appears >3 times

### Step 6 — Verify build
- cargo build passes
- Tree appears in UI
- Collapse state preserved
- Empty state shown before first eval
