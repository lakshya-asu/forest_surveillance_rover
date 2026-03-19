# KiCad Project Notes

This directory now contains slightly richer KiCad design scaffolding than the initial repo scaffold:

- `forest-rover.kicad_sch` includes a structured top-level block floorplan with named functional regions and capture notes
- `forest-rover.kicad_pcb` includes a 4-layer stackup declaration, named nets, and a board-floorplan overlay
- `sym-lib-table` still points to built-in KiCad symbol libraries where possible

These files are still intentionally non-fabrication design scaffolds. The next real hardware pass should replace them with:

1. A proper hierarchical multi-sheet schematic
2. Annotated symbols with footprints and pin mapping
3. Imported netlist / Update PCB from Schematic
4. Real footprints, zones, tracks, and DRC-clean layout data
