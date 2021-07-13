<Qucs Schematic 0.0.19>
<Properties>
  <View=0,-60,1694,1092,1.94872,781,445>
  <Grid=10,10,1>
  <DataSet=Top-Band-LPF.dat>
  <DataDisplay=Top-Band-LPF.dpl>
  <OpenDisplay=1>
  <Script=Top-Band-LPF.m>
  <RunScript=0>
  <showFrame=0>
  <FrameText0=Title>
  <FrameText1=Drawn By:>
  <FrameText2=Date:>
  <FrameText3=Revision:>
</Properties>
<Symbol>
</Symbol>
<Components>
  <Pac P1 1 630 330 18 -26 0 1 "1" 1 "50 Ohm" 1 "0 dBm" 0 "1 GHz" 0 "26.85" 0>
  <GND * 1 630 360 0 0 0 0>
  <GND * 1 740 360 0 0 0 0>
  <L L1 1 810 250 -26 10 0 0 "47uH" 1 "" 0>
  <GND * 1 880 360 0 0 0 0>
  <Pac P2 1 990 330 18 -26 0 1 "2" 1 "50 Ohm" 1 "0 dBm" 0 "1 GHz" 0 "26.85" 0>
  <GND * 1 990 360 0 0 0 0>
  <.SP SP1 1 640 430 0 67 0 0 "log" 1 "20kHz" 1 "300kHz" 1 "201" 1 "no" 0 "1" 0 "2" 0 "no" 0 "no" 0>
  <Eqn Eqn1 1 860 440 -28 15 0 0 "dBS21=dB(S[2,1])" 1 "dBS11=dB(S[1,1])" 1 "yes" 0>
  <C C1 1 740 330 17 -26 0 1 "33nF" 1 "" 0 "neutral" 0>
  <C C2 1 880 330 17 -26 0 1 "33nF" 1 "" 0 "neutral" 0>
</Components>
<Wires>
  <630 250 630 300 "" 0 0 0 "">
  <630 250 740 250 "" 0 0 0 "">
  <740 250 740 300 "" 0 0 0 "">
  <880 250 880 300 "" 0 0 0 "">
  <740 250 780 250 "" 0 0 0 "">
  <840 250 880 250 "" 0 0 0 "">
  <990 250 990 300 "" 0 0 0 "">
  <880 250 990 250 "" 0 0 0 "">
</Wires>
<Diagrams>
</Diagrams>
<Paintings>
  <Text 970 430 12 #000000 0 "Chebyshev low-pass filter \n 200kHz cutoff, pi-type, \n impedance matching 50 Ohm">
</Paintings>
