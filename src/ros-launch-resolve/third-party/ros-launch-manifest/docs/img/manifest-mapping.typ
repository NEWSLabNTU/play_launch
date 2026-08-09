// Launch file → Manifest mapping diagram
// Blocks aligned vertically: arg→args, node+group→nodes, (empty)→topics, include→includes
// Block highlight: full-width colored background per section
// Text color: red for if= and remap wiring (conditions + topic names)
// Compile: typst compile manifest-mapping.typ manifest-mapping.png --format png --ppi 200

#set page(width: 620pt, height: auto, margin: 12pt)
#set text(font: "DejaVu Sans", size: 9pt)

#let code-font = "DejaVu Sans Mono"
#let box-bg = rgb("#f8f9fa")
#let dim = rgb("#999999")
#let cs = 7.5pt
#let red = rgb("#c62828")

// Block colors
#let c-arg = rgb("#e8f0fe")       // blue
#let c-node = rgb("#e6f4ea")      // green
#let c-topic = rgb("#fef7cd")     // yellow
#let c-include = rgb("#f3e8fd")   // purple
#let c-none = box-bg
#let c-gap = white                // gap between blocks

#let xml-color = rgb("#0d47a1")
#let attr-color = rgb("#6a1b9a")
#let yaml-k = rgb("#1b5e20")
#let yaml-v = rgb("#4e342e")

#let lw = 290pt
#let rw = 280pt

// Full-width block line
#let ln(color, indent: 0pt, body) = block(
  width: 100%, fill: color, inset: (x: 4pt, y: 1.5pt), above: 0pt, below: 0pt,
  { h(indent); body }
)
// Gap line (white separator between blocks)
#let gap() = block(width: 100%, fill: c-gap, inset: (x: 4pt, y: 1pt), above: 0pt, below: 0pt, [])

// Title
#align(center, text(size: 13pt, weight: "bold")[From Launch Files to Manifests])
#v(4pt)
#align(center, text(size: 8.5pt, fill: rgb("#5f6368"))[
  Matching block colors show how launch file sections map to manifest sections.
])
#v(6pt)

// Legend
#align(center)[
  #box(fill: c-arg, inset: (x: 6pt, y: 2pt), radius: 2pt, text(size: 7pt)[args])
  #h(6pt)
  #box(fill: c-node, inset: (x: 6pt, y: 2pt), radius: 2pt, text(size: 7pt)[nodes])
  #h(6pt)
  #box(fill: c-topic, inset: (x: 6pt, y: 2pt), radius: 2pt, text(size: 7pt)[topics])
  #h(6pt)
  #box(fill: c-include, inset: (x: 6pt, y: 2pt), radius: 2pt, text(size: 7pt)[includes])
  #h(6pt)
  #box(fill: white, inset: (x: 6pt, y: 2pt), radius: 2pt, stroke: 0.5pt + red, text(fill: red, size: 7pt)[conditions])
]
#v(8pt)

#grid(
  columns: (lw, 1fr, rw),
  gutter: 0pt,

  // ── LEFT: Launch file ──
  {
    text(size: 10pt, fill: xml-color, weight: "bold")[control.launch.xml]
    v(2pt)
    text(size: 7.5pt, fill: rgb("#5f6368"))[
      Declares _what to run_ and _how to wire names_. \
      Topics, types, QoS are implicit in source code.
    ]
    v(4pt)
    rect(radius: 6pt, width: 100%, stroke: 0.5pt + rgb("#dadce0"))[
      #set block(spacing: 0pt)

      // ── arg block (blue) ──
      #ln(c-arg)[
        #text(fill: xml-color, font: code-font, size: cs)[\<arg ]#text(fill: attr-color, font: code-font, size: cs)[name="launch\_validator"]#text(fill: xml-color, font: code-font, size: cs)[/\>]
      ]

      #gap()

      // ── node + group block (green) ──
      // <remap> lines highlighted in yellow (maps to topics:)
      #ln(c-node)[
        #text(fill: xml-color, font: code-font, size: cs)[\<node ]#text(fill: attr-color, font: code-font, size: cs)[pkg="controller"]#text(fill: xml-color, font: code-font, size: cs)[\>]
      ]
      #ln(c-node, indent: 10pt)[
        #box(fill: c-topic, radius: 2pt, inset: (x: 2pt, y: 0.5pt))[#text(fill: xml-color, font: code-font, size: cs)[\<remap ]#text(fill: attr-color, font: code-font, size: cs)[from="~/cmd" to="/control/cmd"]#text(fill: xml-color, font: code-font, size: cs)[/\>]]
      ]
      #ln(c-node)[
        #text(fill: dim, font: code-font, size: cs)[\</node\>]
      ]
      #ln(c-node)[
        #text(fill: xml-color, font: code-font, size: cs)[\<group ]#text(fill: red, font: code-font, size: cs)[if="\$(var launch\_validator)"]#text(fill: xml-color, font: code-font, size: cs)[\>]
      ]
      #ln(c-node, indent: 10pt)[
        #text(fill: xml-color, font: code-font, size: cs)[\<node ]#text(fill: attr-color, font: code-font, size: cs)[pkg="validator"]#text(fill: xml-color, font: code-font, size: cs)[/\>]
      ]
      #ln(c-node, indent: 10pt)[
        #box(fill: c-topic, radius: 2pt, inset: (x: 2pt, y: 0.5pt))[#text(fill: xml-color, font: code-font, size: cs)[\<remap ]#text(fill: attr-color, font: code-font, size: cs)[from="~/input" to="/control/cmd"]#text(fill: xml-color, font: code-font, size: cs)[/\>]]
      ]
      #ln(c-node)[
        #text(fill: dim, font: code-font, size: cs)[\</group\>]
      ]
      #ln(c-node)[#h(1pt)]
      #ln(c-node)[#h(1pt)]
      #ln(c-node)[#h(1pt)]
      #ln(c-node)[#h(1pt)]

      #gap()

      // ── include block (purple) ──
      #ln(c-include)[
        #text(fill: xml-color, font: code-font, size: cs)[\<include ]#text(fill: attr-color, font: code-font, size: cs)[file="system.launch.xml"]#text(fill: xml-color, font: code-font, size: cs)[/\>]
      ]
    ]
  },

  [],

  // ── RIGHT: Manifest file ──
  {
    text(size: 10pt, fill: yaml-k, weight: "bold")[control.yaml]
    v(2pt)
    text(size: 7.5pt, fill: rgb("#5f6368"))[
      Declares _what communicates_ and _at what quality_. \
      Topics are first-class with type, QoS, rate, latency.
    ]
    v(4pt)
    rect(radius: 6pt, width: 100%, stroke: 0.5pt + rgb("#dadce0"))[
      #set block(spacing: 0pt)

      // ── args block (blue) ──
      #ln(c-arg)[
        #text(fill: yaml-k, font: code-font, size: cs, weight: "bold")[args:]
      ]
      #ln(c-arg, indent: 6pt)[
        #text(fill: yaml-k, font: code-font, size: cs, weight: "bold")[launch\_validator:]
      ]
      #ln(c-arg, indent: 14pt)[
        #text(fill: yaml-k, font: code-font, size: cs, weight: "bold")[type: ]#text(fill: yaml-v, font: code-font, size: cs)[bool]
      ]

      // ── nodes block (green) ──
      #ln(c-node)[
        #text(fill: yaml-k, font: code-font, size: cs, weight: "bold")[nodes:]
      ]
      #ln(c-node, indent: 6pt)[
        #text(fill: yaml-k, font: code-font, size: cs, weight: "bold")[controller:]
      ]
      #ln(c-node, indent: 14pt)[
        #text(fill: yaml-k, font: code-font, size: cs, weight: "bold")[pub:]
      ]
      #ln(c-node, indent: 22pt)[
        #text(fill: yaml-k, font: code-font, size: cs, weight: "bold")[cmd:]
      ]
      #ln(c-node, indent: 30pt)[
        #text(fill: yaml-k, font: code-font, size: cs, weight: "bold")[min\_rate\_hz: ]#text(fill: yaml-v, font: code-font, size: cs)[30]
      ]
      #ln(c-node, indent: 6pt)[
        #text(fill: yaml-k, font: code-font, size: cs, weight: "bold")[validator:]
      ]
      #ln(c-node, indent: 14pt)[
        #text(fill: red, font: code-font, size: cs, weight: "bold")[if: ]#text(fill: red, font: code-font, size: cs)[\$(var launch\_validator)]
      ]
      #ln(c-node, indent: 14pt)[
        #text(fill: yaml-k, font: code-font, size: cs, weight: "bold")[sub:]
      ]
      #ln(c-node, indent: 22pt)[
        #text(fill: yaml-k, font: code-font, size: cs, weight: "bold")[input:]
      ]
      #ln(c-node, indent: 30pt)[
        #text(fill: yaml-k, font: code-font, size: cs, weight: "bold")[min\_rate\_hz: ]#text(fill: yaml-v, font: code-font, size: cs)[30]
      ]

      // ── topics block (yellow) ──
      #ln(c-topic)[
        #text(fill: yaml-k, font: code-font, size: cs, weight: "bold")[topics:]
      ]
      #ln(c-topic, indent: 6pt)[
        #text(fill: yaml-k, font: code-font, size: cs, weight: "bold")[control\_cmd:]
      ]
      #ln(c-topic, indent: 14pt)[
        #text(fill: yaml-k, font: code-font, size: cs, weight: "bold")[type: ]#text(fill: yaml-v, font: code-font, size: cs)[autoware\_control\_msgs/msg/Control]
      ]
      #ln(c-topic, indent: 14pt)[
        #text(fill: yaml-k, font: code-font, size: cs, weight: "bold")[pub: ]#text(fill: yaml-v, font: code-font, size: cs)[\[controller/cmd\]]
      ]
      #ln(c-topic, indent: 14pt)[
        #text(fill: yaml-k, font: code-font, size: cs, weight: "bold")[sub: ]#text(fill: yaml-v, font: code-font, size: cs)[\[validator/input\]]
      ]
      #ln(c-topic, indent: 14pt)[
        #text(fill: yaml-k, font: code-font, size: cs, weight: "bold")[rate\_hz: ]#text(fill: yaml-v, font: code-font, size: cs)[30]
      ]

      // ── includes block (purple) ──
      #ln(c-include)[
        #text(fill: yaml-k, font: code-font, size: cs, weight: "bold")[includes:]
      ]
      #ln(c-include, indent: 6pt)[
        #text(fill: yaml-k, font: code-font, size: cs, weight: "bold")[system:]
      ]
      #ln(c-include, indent: 14pt)[
        #text(fill: yaml-k, font: code-font, size: cs, weight: "bold")[manifest: ]#text(fill: yaml-v, font: code-font, size: cs)[system.yaml]
      ]
    ]
  }
)
