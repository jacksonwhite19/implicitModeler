# Phase 5: Export & Integration — Remaining Tasks

## Status

**Completed**:
- ✅ STL Export (binary format)
- ✅ OBJ Export (with normals)

**Remaining**:
- STEP file export (complex, potentially skip for MVP)
- Batch/headless mode for automation

---

## Stage A: Batch/Headless Mode (3 steps) ✅ COMPLETE

### A1. Add CLI argument parsing
- [x] Use `clap` crate for argument parsing
- [x] Support `--headless` flag
- [x] Support `--script <path>` to run script file
- [x] Support `--output <path>` for export location
- [x] Support `--format <stl|obj>` for export format
- **Verify**: CLI args parsed correctly ✅

### A2. Implement headless execution
- [x] Skip GUI initialization in headless mode
- [x] Load and execute script file
- [x] Extract mesh
- [x] Export to specified format and location
- [x] Print errors to stderr, success to stdout
- **Verify**: Can run from command line ✅

### A3. Add batch processing
- [x] Support `--batch <dir>` to process all .rhai files
- [x] Export each to STL/OBJ with same base name
- [x] Generate summary report
- [x] Return exit code (0 = success, 1 = errors)
- **Verify**: Batch processing works ✅

---

## Stage B: Advanced Export Options (2 steps)

### B1. Export settings dialog
- [ ] Add "Export Settings" button
- [ ] Options: mesh resolution, format, file name
- [ ] Preview triangle count before export
- [ ] Remember last settings
- **Verify**: Export customization works

### B2. Export validation
- [ ] Check mesh is manifold (no holes)
- [ ] Warn if mesh has issues
- [ ] Option to auto-repair common issues
- [ ] Show mesh statistics before export
- **Verify**: Exports are clean

---

## Stage C: STEP Export (Optional, Complex)

**Note**: STEP (ISO 10303) export is significantly more complex than STL/OBJ. It requires:
- B-Rep conversion (our SDFs are implicit, STEP needs explicit boundaries)
- STEP file format encoding (complex standard)
- Potentially third-party libraries (opencascade)

**Recommendation**: Defer this or skip for MVP. Most 3D printing and basic CAD workflows work fine with STL/OBJ.

If needed:
- [ ] Research STEP conversion libraries
- [ ] Prototype B-Rep extraction from SDF
- [ ] Implement STEP file writer
- [ ] Test with CAD software

---

## Summary

| Stage | Steps | Priority | Description                    |
| ----- | ----- | -------- | ------------------------------ |
| A     | 3     | High     | Batch/headless mode            |
| B     | 2     | Medium   | Advanced export options        |
| C     | ?     | Low      | STEP export (complex, optional)|
| **Total** | **5+** |      | **Phase 5 remaining**       |

## Implementation Priority

1. **Stage A** (Batch Mode) - Enables automation and CI/CD integration
2. **Stage B** (Export Options) - Polish for existing export features
3. **Stage C** (STEP) - Skip unless specifically needed

## Notes

- Phase 5 is mostly complete thanks to Phase 3 work
- Batch mode is the highest value remaining feature
- STEP export may not be worth the complexity for an MVP
