// Manifest scope and wiring model diagram
// Shows: scopes as boxes, nodes with endpoint ports (vertical), topic/service wiring
// Compile: typst compile manifest-model.typ manifest-model.svg --format svg

#set page(width: 680pt, height: auto, margin: 12pt)
#set text(font: "DejaVu Sans", size: 8.5pt)

#let code-font = "DejaVu Sans Mono"
#let scope-border = rgb("#dadce0")
#let node-bg = rgb("#e8f0fe")        // blue — nodes
#let topic-bg = rgb("#fef7cd")       // yellow — topics (distinct from nodes)
#let service-bg = rgb("#fce8e6")     // red-ish — services
#let port-pub = rgb("#34a853")
#let port-sub = rgb("#ea4335")
#let port-srv = rgb("#e37400")
#let yaml-k = rgb("#1b5e20")
#let yaml-v = rgb("#4e342e")
#let dim = rgb("#5f6368")

#let code(body) = text(font: code-font, size: 7.5pt, body)
#let label-text(body) = text(size: 7pt, fill: dim, body)

#let pub-badge = box(fill: port-pub, radius: 2pt, inset: (x: 3pt, y: 1.5pt), text(fill: white, size: 6pt, weight: "bold")[pub])
#let sub-badge = box(fill: port-sub, radius: 2pt, inset: (x: 3pt, y: 1.5pt), text(fill: white, size: 6pt, weight: "bold")[sub])
#let srv-badge = box(fill: port-srv, radius: 2pt, inset: (x: 3pt, y: 1.5pt), text(fill: white, size: 6pt, weight: "bold")[srv])
#let cli-badge = box(fill: port-srv, radius: 2pt, inset: (x: 3pt, y: 1.5pt), text(fill: white, size: 6pt, weight: "bold")[cli])

// Title
#align(center, text(size: 13pt, weight: "bold")[Manifest Model: Scopes, Nodes, and Wiring])
#v(4pt)
#align(center, text(size: 8.5pt, fill: dim)[
  Each manifest describes one scope. Scopes contain nodes, topics, services, and child scopes.
])
#v(6pt)

// Legend
#align(center)[
  #box(width: 12pt, height: 8pt, fill: node-bg, radius: 2pt, baseline: 1pt)
  #h(2pt) #text(size: 7pt)[node] #h(10pt)
  #pub-badge #h(2pt) #text(size: 7pt)[publisher] #h(10pt)
  #sub-badge #h(2pt) #text(size: 7pt)[subscriber] #h(10pt)
  #box(width: 12pt, height: 8pt, fill: topic-bg, stroke: 0.5pt + rgb("#e0c000"), radius: 2pt, baseline: 1pt)
  #h(2pt) #text(size: 7pt)[topic] #h(10pt)
  #box(width: 12pt, height: 8pt, fill: service-bg, stroke: 0.5pt + port-srv, radius: 2pt, baseline: 1pt)
  #h(2pt) #text(size: 7pt)[service]
]
#v(10pt)

// ═══════════════════════════════════════════════════════════
// PART 1: Topic wiring — perception.yaml
// ═══════════════════════════════════════════════════════════

#rect(stroke: 1.5pt + scope-border, radius: 8pt, inset: 12pt, width: 100%)[
  #text(size: 10pt, weight: "bold")[perception.yaml]
  #h(6pt) #label-text[scope: /perception]
  #v(4pt)
  #label-text[endpoints of perception.yaml — wired by parent scope]
  #v(3pt)
  #sub-badge #h(3pt) #code[pointcloud]
  #v(3pt)
  #sub-badge #h(3pt) #code[vector\_map]
  #v(3pt)
  #pub-badge #h(3pt) #code[predicted\_objects]
  #v(8pt)

  // Two child scopes
  #grid(
    columns: (1fr, 1fr),
    gutter: 12pt,

    // LEFT: tracking
    rect(stroke: 1pt + scope-border, radius: 6pt, inset: 10pt)[
      #text(size: 9pt, weight: "bold")[tracking.yaml]
      #h(4pt) #label-text[include: tracking/]
      #v(6pt)

      // Scope interface endpoints (before node — these are the scope's ports)
      #sub-badge #h(3pt) #code[detected\_objects]
      #v(3pt)
      #pub-badge #h(3pt) #code[tracked\_objects]

      #v(8pt)

      // Node
      #rect(fill: node-bg, radius: 4pt, inset: 8pt, width: 100%)[
        #code[multi\_object\_tracker]
        #v(4pt)
        #sub-badge #h(3pt) #code[detected]
        #v(3pt)
        #pub-badge #h(3pt) #code[tracked]
      ]
    ],

    // RIGHT: prediction
    rect(stroke: 1pt + scope-border, radius: 6pt, inset: 10pt)[
      #text(size: 9pt, weight: "bold")[prediction.yaml]
      #h(4pt) #label-text[include: prediction/]
      #v(6pt)

      // Scope interface endpoints
      #sub-badge #h(3pt) #code[objects]
      #v(3pt)
      #sub-badge #h(3pt) #code[vector\_map]
      #v(3pt)
      #pub-badge #h(3pt) #code[predicted\_objects]

      #v(8pt)

      // Node
      #rect(fill: node-bg, radius: 4pt, inset: 8pt, width: 100%)[
        #code[map\_based\_prediction]
        #v(4pt)
        #sub-badge #h(3pt) #code[tracked]
        #v(3pt)
        #sub-badge #h(3pt) #code[vector\_map] #label-text[(state)]
        #v(3pt)
        #pub-badge #h(3pt) #code[predicted]
      ]
    ],
  )

  #v(10pt)

  // Topic wiring — yellow background
  #rect(fill: topic-bg, stroke: 0.5pt + rgb("#e0c000"), radius: 4pt, inset: 8pt, width: 100%)[
    #text(fill: rgb("#7a6c00"), weight: "bold", size: 8pt)[topics:] #h(4pt) #label-text[parent scope wires child endpoints via topics]
    #v(3pt)
    #text(fill: yaml-k, font: code-font, size: 7.5pt, weight: "bold")[tracked\_objects:]
    #v(1pt)
    #h(8pt)#text(fill: yaml-k, font: code-font, size: 7.5pt, weight: "bold")[type: ]#text(fill: yaml-v, font: code-font, size: 7.5pt)[TrackedObjects]
    #v(1pt)
    #h(8pt)#text(fill: yaml-k, font: code-font, size: 7.5pt, weight: "bold")[pub: ]#text(fill: yaml-v, font: code-font, size: 7.5pt)[\[tracking/tracked\_objects\]]
    #v(1pt)
    #h(8pt)#text(fill: yaml-k, font: code-font, size: 7.5pt, weight: "bold")[sub: ]#text(fill: yaml-v, font: code-font, size: 7.5pt)[\[prediction/objects\]]
    #v(1pt)
    #h(8pt)#text(fill: yaml-k, font: code-font, size: 7.5pt, weight: "bold")[rate\_hz: ]#text(fill: yaml-v, font: code-font, size: 7.5pt)[10]
  ]

]

#v(16pt)

// ═══════════════════════════════════════════════════════════
// PART 2: Service wiring — system.yaml
// ═══════════════════════════════════════════════════════════

#align(center, text(size: 11pt, weight: "bold")[Service Wiring Across Scopes])
#v(8pt)

#rect(stroke: 1.5pt + scope-border, radius: 8pt, inset: 12pt, width: 100%)[
  #text(size: 10pt, weight: "bold")[system.yaml]
  #h(6pt) #label-text[scope: /system — parent wires cross-scope services]
  #v(8pt)

  #grid(
    columns: (1fr, 1fr),
    gutter: 12pt,

    // mrm_handler
    rect(stroke: 1pt + scope-border, radius: 6pt, inset: 10pt)[
      #text(size: 9pt, weight: "bold")[mrm\_handler.yaml]
      #h(4pt) #label-text[include]
      #v(6pt)

      // Scope interface
      #cli-badge #h(3pt) #code[comfortable\_stop]
      #v(3pt)
      #cli-badge #h(3pt) #code[emergency\_stop]

      #v(8pt)

      #rect(fill: node-bg, radius: 4pt, inset: 8pt, width: 100%)[
        #code[mrm\_handler]
        #v(4pt)
        #cli-badge #h(3pt) #code[comfortable\_stop\_operate]
        #v(3pt)
        #cli-badge #h(3pt) #code[emergency\_stop\_operate]
      ]
    ],

    // mrm operators
    rect(stroke: 1pt + scope-border, radius: 6pt, inset: 10pt)[
      #text(size: 9pt, weight: "bold")[mrm\_comfortable\_stop.yaml]
      #h(4pt) #label-text[include]
      #v(6pt)

      // Scope interface
      #srv-badge #h(3pt) #code[operate]

      #v(8pt)

      #rect(fill: node-bg, radius: 4pt, inset: 8pt, width: 100%)[
        #code[mrm\_comfortable\_stop\_operator]
        #v(4pt)
        #srv-badge #h(3pt) #code[operate]
      ]
      #v(8pt)
      #text(size: 9pt, weight: "bold")[mrm\_emergency\_stop.yaml]
      #h(4pt) #label-text[include]
      #v(6pt)
      #srv-badge #h(3pt) #code[operate]
      #v(8pt)
      #rect(fill: node-bg, radius: 4pt, inset: 8pt, width: 100%)[
        #code[mrm\_emergency\_stop\_operator]
        #v(4pt)
        #srv-badge #h(3pt) #code[operate]
      ]
    ],
  )

  #v(10pt)

  // Service wiring — red-ish background
  #rect(fill: service-bg, stroke: 0.5pt + port-srv, radius: 4pt, inset: 8pt, width: 100%)[
    #text(fill: port-srv, weight: "bold", size: 8pt)[services:] #h(4pt) #label-text[parent scope wires cli #sym.arrow.r srv across children]
    #v(3pt)
    #text(fill: yaml-k, font: code-font, size: 7.5pt, weight: "bold")[comfortable\_stop\_operate:]
    #v(1pt)
    #h(8pt)#text(fill: yaml-k, font: code-font, size: 7.5pt, weight: "bold")[type: ]#text(fill: yaml-v, font: code-font, size: 7.5pt)[tier4\_system\_msgs/srv/OperateMrm]
    #v(1pt)
    #h(8pt)#text(fill: yaml-k, font: code-font, size: 7.5pt, weight: "bold")[server: ]#text(fill: yaml-v, font: code-font, size: 7.5pt)[\[mrm\_comfortable\_stop/operate\]]
    #v(1pt)
    #h(8pt)#text(fill: yaml-k, font: code-font, size: 7.5pt, weight: "bold")[client: ]#text(fill: yaml-v, font: code-font, size: 7.5pt)[\[mrm\_handler/comfortable\_stop\]]
  ]
]
