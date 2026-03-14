# Phase 4: UI Features — Task Plan

## Goal
Add productivity features to make the editor more powerful and user-friendly.

## Legend
- [ ] Pending
- [x] Complete
- [>] In Progress

---

## Stage A: Save/Load Projects (3 steps) ✅ COMPLETE

### A1. Define project file format
- [x] Create `.icad` JSON format with script content and metadata
- [x] Include version, timestamp, camera position
- [x] Store mesh resolution and rendering settings
- **Verify**: Format is well-defined ✅

### A2. Implement save functionality
- [x] Add "Save" button and Ctrl+S shortcut
- [x] Use native file dialog to choose location
- [x] Serialize current state to JSON
- [x] Show success/error feedback
- **Verify**: Can save project ✅

### A3. Implement load functionality
- [x] Add "Load" button and Ctrl+O shortcut
- [x] Use native file dialog to choose file
- [x] Deserialize and restore state
- [x] Restore camera position and settings
- **Verify**: Can load saved project ✅

---

## Stage B: Script Auto-Save (2 steps)

### B1. Implement auto-save
- [ ] Save script to temp file every 30 seconds
- [ ] Save on successful execution
- [ ] Use app data directory for auto-saves
- **Verify**: Auto-save works

### B2. Restore from auto-save
- [ ] On startup, check for auto-save
- [ ] Prompt user to restore if found
- [ ] Clear auto-save on explicit save
- **Verify**: Can recover from crash

---

## Stage C: Undo/Redo (3 steps) ✅ COMPLETE

### C1. Implement history stack
- [x] Track script text changes
- [x] Limit history to last 50 states
- [x] Store in memory-efficient way
- **Verify**: History captured ✅

### C2. Add undo functionality
- [x] Ctrl+Z keyboard shortcut
- [x] Restore previous script state
- [x] Update execution if needed
- **Verify**: Undo works ✅

### C3. Add redo functionality
- [x] Ctrl+Y or Ctrl+Shift+Z shortcut
- [x] Move forward in history
- [x] Clear redo stack on new edit
- **Verify**: Redo works ✅

---

## Stage D: Quick Insert Menu (3 steps)

### D1. Create insert menu
- [ ] Add "Insert" dropdown button
- [ ] Categories: Primitives, Operations, Transforms
- [ ] Show syntax for each item
- **Verify**: Menu appears

### D2. Implement code insertion
- [ ] Insert template at cursor position
- [ ] Use placeholders for parameters (e.g., `<radius>`)
- [ ] Maintain cursor position after insert
- **Verify**: Code inserts correctly

### D3. Add common snippets
- [ ] Bracket pattern (subtract hole from base)
- [ ] Array pattern (linear repeat)
- [ ] Blend pattern (smooth union)
- **Verify**: Snippets useful

---

## Stage E: Syntax Highlighting (2 steps)

### E1. Basic syntax highlighting
- [ ] Highlight keywords (let, if, for, etc.)
- [ ] Highlight function names
- [ ] Highlight numbers and strings
- [ ] Use egui's built-in support or custom
- **Verify**: Code is easier to read

### E2. Error underlining
- [ ] Underline syntax errors in red
- [ ] Show error on hover
- [ ] Update as user types (with debounce)
- **Verify**: Errors visible inline

---

## Stage F: Recent Files (2 steps)

### F1. Track recent files
- [ ] Store list of recent file paths
- [ ] Limit to 10 most recent
- [ ] Save to preferences file
- **Verify**: Recent files tracked

### F2. Recent files menu
- [ ] Add "Recent" submenu under File
- [ ] Show file names with paths
- [ ] Click to open
- **Verify**: Quick access works

---

## Summary

| Stage | Steps | Description                     |
| ----- | ----- | ------------------------------- |
| A     | 3     | Save/load project files         |
| B     | 2     | Auto-save and recovery          |
| C     | 3     | Undo/redo functionality         |
| D     | 3     | Quick insert menu               |
| E     | 2     | Syntax highlighting             |
| F     | 2     | Recent files list               |
| **Total** | **15** | **Phase 4 tasks**           |

## Priority Order

1. **Stage A** (Save/Load) - Essential for real projects
2. **Stage B** (Auto-save) - Prevents data loss
3. **Stage C** (Undo/Redo) - Major productivity boost
4. **Stage D** (Quick Insert) - Speeds up workflow
5. **Stage F** (Recent Files) - Convenience feature
6. **Stage E** (Syntax Highlighting) - Nice to have

## Notes

- Focus on stages A, B, C first (essential features)
- Stages D, E, F are productivity enhancements
- All features should feel native and polished
