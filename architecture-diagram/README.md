# Architecture diagram

A web page that draws the robot's modules and the data that flows between them,
from data files. It is for reviewing the design, now and as planned
([DATA_FLOW_PLAN.md](../doc/DATA_FLOW_PLAN.md), [LINE_MAP_PLAN.md](../doc/LINE_MAP_PLAN.md)),
and for commenting on it. First iteration built 8 Nov: the modules on the first
flows and those flows. This file is the page; [data/README.md](data/README.md) is
the data (words, scope, shapes, the check, how to fill it).

First iteration: small on purpose. Add only what a review shows is missing.

## Data and view are apart

```
architecture-diagram/
  README.md           # this: the page, how to run it
  package.json        # d3, typescript, vite; exact versions, newest at install
  tsconfig.json, vite.config.ts
  data/               # the architecture: read this, never the view
    README.md, schema.ts, modules/*.json, flows.json
  tools/
    resolve.ts        # npm run resolve: checks the data against the code, writes generated/
  generated/          # code.json: each reference's line and declaration text; git-ignored
  view/               # the web page; only draws data/ and generated/
    index.html, style.css
    src/data.ts       # loads and indexes data/ and generated/
    src/layout.ts     # where boxes, ribbons and nodes go (pure)
    src/draw.ts       # the canvas (d3)
    src/panels.ts     # the flow list and the side bar
    src/main.ts       # keeps them in step; positions, the address
```

- **`data/`** holds everything about the architecture and links into the code. An
  agent reading a comment from Daniel reads only `data/` and the code it points at.
- **`view/`** knows nothing about the robot: no module, flow or origin is written in
  it. It loads `data/modules/*.json` by pattern and takes the origins' order from
  `data/schema.ts`.
- **`tools/resolve.ts`** is the only code that reads both the data and the robot's
  code.

## What the page shows

- **Module boxes** on a canvas, one per module, draggable: the module's name and its
  members' names. No types or parameters in the box.
- **Ribbons** for the flows. A ribbon ends at the member it goes through (its row in
  the box).
- **Ribbon width:** the data's `bytes` (one message), on a log scale (from 8 B for
  pitch and roll to ~20 KB for a snapshot). Bytes, not bytes per second: the rate is
  a label on the ribbon.
- **Change nodes:** small circles where the data changes; the ribbon after one has
  the new width. A node made by a member sits on that member's row.
- **Subflows** start at the row where they leave the parent, beside its ribbon. A
  ribbon is as wide as what the code really hands over: `tilt-stop` is fat up to
  `drive_update`, because `drive_update` gets all of `odom_t` and picks pitch and
  roll inside (the pick's circle sits on its row).
- **Built or planned:** planned things are dashed; ones the plan removes are greyed.
- **A list of flows** (left): name, id, description, from → to.
- **The side bar** (right), on a click:
  - a box: each member's declaration and comment, as in the code;
  - a ribbon or a flow in the list: description, rate, each step's type with its
    declaration and bytes, each node;
  - a node: what changes, the type before and after, the code.
- **Links to the code** on everything: clicking one opens the file at the line in VS
  Code (`vscode://file/<repo>/<path>:<line>`; the repo's path and the line come from
  `generated/code.json`).
- **The address holds the selection:** `#flow=tilt-stop`, `#module=picoB.drive`,
  `#node=odometry/4` (a step's index from 0), so a link shows one thing.

## What it is for

- See the modules and their interfaces at a glance; spot shallow modules, leaks,
  one rule in two places.
- Follow one piece of data from where it is made to where it is used, and see where
  it is cut down, changed or joined.
- Compare what is built with what the plans say.
- **Find what is wrong while gathering the data:** dead or unneeded code, data that
  flows nowhere or is ignored, flow errors (wrong rate, wrong type, a step that
  skips a check), names that differ for one thing. Filling the data means reading
  every hand-over, so this is the cheapest moment to see them. How:
  [data/README.md](data/README.md), "Findings".
- Daniel comments by naming flows, modules and nodes; the page has no comments of
  its own.

## Drawing

- **Groups of boxes by origin**, left to right in `schema.ts`'s order: PicoB, common,
  PicoA, PC, browser. common's boxes (C built into both Picos, among them the
  PicoA–PicoB link) are drawn once, between the two Picos.
- **Inside a group**, boxes stack top to bottom in the order they first appear in
  `flows.json`; boxes on no flow go at the bottom. Dragging fixes the rest;
  positions are kept in the browser (localStorage), a button resets them.
- **Ribbons** leave and enter a box on the side that faces the other box, so data
  going right to left (the answer: PicoA → PC → browser) draws cleanly. Boxes in one
  column: the ribbon leaves and enters on the right, a loop. Several ribbons at one
  row sit side by side, scaled down to fit the row.
- **Two steps in a row in the same box** (pack then unpack in `tof_format`) get no
  ribbon between them; their nodes sit on their rows.
- d3: ribbons are cubic curves (stroke as wide as the ribbon, a wider invisible one
  to click), boxes are SVG groups moved by `d3.drag`, the canvas by `d3.zoom`; the
  first view fits the selected flow's boxes, or everything. Not `d3.ribbon`: it draws chords around a circle.
- Colours: the dataviz skill's reference palette, light and dark; one categorical
  colour per top-level flow in `flows.json`'s order, a subflow in its parent's.

## Run

In `architecture-diagram/`:

```sh
npm install
npm run dev        # resolve, then the page at http://127.0.0.1:5173/ (#flow=tilt-stop for one flow)
npm run resolve    # check the data against the code
npm run typecheck
```

- `npm run dev` shows the page even when the check fails; what failed is listed on
  the page.
- `tools/lint.sh` runs the check and the typecheck: it needs `npm install` here too.
- Libraries come from npm, none from a CDN: d3 (newest, 7.x now), TypeScript
  (newest, 7.x now), Vite.

## Open

- **`bytes` is written by hand** and not checked. Later, if it drifts, the check
  could build the `sizeof` program itself.
- **The tilt stop** ends at `drive_update` for now. Later: motors off →
  `DRIVE_TILTED` → `brain` → `STOP_TILT` to PicoA → the move ends with `tilt`.
