"""Fixed vector layout for the same 18 edges as the Markdown architecture graph."""
from pathlib import Path
from html import escape
parts=['''<svg xmlns="http://www.w3.org/2000/svg" width="770" height="765" viewBox="0 0 770 765"><defs><marker id="arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8 Z" fill="#557787"/></marker></defs><rect width="770" height="765" fill="white"/><style>text{font-family:Arial,sans-serif;fill:#18384c}.edge{fill:none;stroke:#557787;stroke-width:1.4;marker-end:url(#arrow)}.feedback{stroke-dasharray:5 3}.label{font-size:12px;paint-order:stroke;stroke:white;stroke-width:5px;stroke-linejoin:round}</style>''']
def edge(d,label=None,x=0,y=0,feedback=False):
 parts.append(f'<path class="edge {"feedback" if feedback else ""}" d="{d}"/>')
 if label: parts.append(f'<text class="label" x="{x}" y="{y}" text-anchor="middle">{escape(label)}</text>')
def node(x,y,w,h,lines,accent=False):
 parts.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="5" fill="{"#e6f1ef" if accent else "#eff5f8"}" stroke="#3d687b"/>')
 for i,line in enumerate(lines):
  yy=y+h/2+(i-(len(lines)-1)/2)*21+6
  parts.append(f'<text x="{x+w/2}" y="{yy}" text-anchor="middle" font-size="{17 if i==0 else 15}" font-weight="{"600" if i==0 else "400"}">{escape(line)}</text>')
edge('M315 65 V85 H220 V105') # S -> P
edge('M455 65 V85 H555 V105') # S -> Q
edge('M220 170 V235') # P -> N
edge('M90 135 H60 V395 H90') # P -> R
edge('M350 155 H390 V470 H652 V520','height map',392,190) # P -> G
edge('M490 170 V215 H410 V485 H480 V520') # Q -> O
edge('M620 170 H710 V485 H685 V520') # Q -> G
edge('M220 295 V360','nav_cmd_vel',265,329) # N -> R
edge('M350 385 H440','cmd_vel',395,375) # R -> C
edge('M290 430 V610 H555 V680','owner request / arm',391,603) # R -> A
edge('M500 425 V460 H480 V520') # C -> O
edge('M620 425 V520') # C -> G
edge('M480 580 V680') # O -> A
edge('M650 580 V680') # G -> A
edge('M440 715 H375 V415 H350','owner ACK',377,657,True) # A -> R
edge('M680 715 H755 V42 H485','JOINTS_CMD',700,33) # A -> S
edge('M285 42 H15 V712 H70') # S -> E
edge('M90 415 H45 V645 H195 V680') # R -> E
node(285,20,200,45,['MuJoCo / S10'])
node(90,105,260,65,['Ray-cast perception','GT odometry / wheel state'])
node(430,105,250,65,['SDK proprioception','Joints / attitude / angular rate'])
node(90,235,260,60,['Waypoint follower','Pursuit / planner / terrain'])
node(90,360,260,70,['Strategy router','Rule-based state machine'],True)
node(440,365,240,60,['SDK velocity interface'])
node(415,520,130,60,['Official actor','57D input'])
node(570,520,165,60,['Gate 16 ONNX','174D base + residual'])
node(440,680,240,65,['Joint owner arbiter','Single actuator command'],True)
node(70,680,250,65,['Evaluator / recorder'])
parts.append('</svg>')
Path('report_assets/figure-1.svg').write_text(''.join(parts))
