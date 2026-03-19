<Qucs Schematic 0.0.24>
<Properties>
  <View=0,0,1600,900,1,0,0>
  <Grid=10,10,1>
  <DataSet=sensor-frontend.dat>
  <DataDisplay=sensor-frontend.dpl>
  <OpenDisplay=1>
  <Script=sensor-frontend.m>
  <RunScript=0>
</Properties>
<Symbol>
</Symbol>
<Components>
  <Lib OP1 1 260 180 10 -26 0 0 "LM358" 1>
  <R R1 1 120 160 -26 15 0 0 "10k" 1 "" 0 "neutral" 0>
  <R R2 1 120 220 -26 15 0 0 "10k" 1 "" 0 "neutral" 0>
  <C C1 1 360 140 -26 17 0 0 "1u" 1 "" 0 "neutral" 0>
  <R R3 1 440 140 -26 15 0 0 "15k" 1 "" 0 "neutral" 0>
  <C C2 1 360 240 -26 17 0 0 "1u" 1 "" 0 "neutral" 0>
  <R R4 1 440 240 -26 15 0 0 "15k" 1 "" 0 "neutral" 0>
  <Lib COMP1 1 620 190 10 -26 0 0 "LM393" 1>
  <Pot VR1 1 720 250 -26 15 0 0 "50k" 1 "" 0 "neutral" 0>
  <Vac V1 1 60 190 18 -26 0 1 "1 V" 1 "1 Hz" 1 "0" 0 "0" 0>
  <GND * 1 60 250 0 0 0 0>
  <GND * 1 120 260 0 0 0 0>
</Components>
<Wires>
  <60 190 90 190 "" 0 0 0 "">
  <90 190 120 190 "" 0 0 0 "">
  <120 160 120 190 "" 0 0 0 "">
  <120 190 120 220 "" 0 0 0 "">
  <120 220 120 260 "" 0 0 0 "">
  <120 190 220 190 "" 0 0 0 "">
  <220 190 260 190 "" 0 0 0 "">
  <300 190 360 190 "" 0 0 0 "">
  <360 140 360 190 "" 0 0 0 "">
  <360 190 360 240 "" 0 0 0 "">
  <360 190 620 190 "" 0 0 0 "">
</Wires>
<Diagrams>
  <AC 1 980 180 420 260 3 #c0c0c0 1 00 1 1e-1 1 1e3 1 0.1 20 0 0 225 1 0 "10 Hz Low-Pass Response" "Frequency (Hz)" "Magnitude (dB)">
</Diagrams>
<Paintings>
  <Text 70 80 14 #000000 0 "PIR front-end: LM358 buffer + 2nd-order Sallen-Key LPF">
  <Text 70 105 12 #000000 0 "Target corner frequency is approximately 10 Hz">
  <Text 70 125 12 #000000 0 "Comparator threshold is adjustable for event detection">
</Paintings>
