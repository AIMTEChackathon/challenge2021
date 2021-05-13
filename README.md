# #AimtecHackathon Challenge Robotík readme.md
Na robota lze posílat příkazy přes seriovou linku přímo z RasPi.
Existují 2 režimy Simple/Advanced.

V simple režimu robot 
  - jezdí vpřed vzad (oba pásy najednou stejným směrem)
  - otáčí se vlevo vpravo (pásy protichůdně)
  - jezdí přednastavenou rychlostí (POZOR: Při malé rychlosti hrozí poškození, protože motory neutáhnou zátěž. Konzultujte s mentorem, pomůžeme vám rychlost vyladit.)
  - pohybuje vidlemi (lift) nahoru dolu o jednu pozici (trvá cca 8s z pozice na zemi do pozice zvednuto), lift má dole koncový spínač, takže je kalibrován na nabrání palety
  - svítí LED páskem v několika režimech

Simple Controls

    ---=== HELP - Simple Controls ===---
    0       .. STOP ALL (motors, lift, lights)
    w,s,a,d .. forward, backward, left, right
    +,-      .. increase/decrease DC motor speed
    e,q     .. lift one floor UP/DOWN
    <space> .. STOP all motors (DC L/R, stepper)
    1,2,3,4 .. light modes [POLICE, GREEN FLASH, BRIGHT WHITE, BLUE BIN 0xAA]
    5       .. lights off
    l,k     .. La(ser)Dis(tance) Measurement ON/OFF
    i       .. toggle debug lights

V advanced režimu lze nastavit zvlášť rychlost a směr každého z pásů L/R.

    ---=== HELP - Advanced Controls ===---
    #STOP#  .. STOP ALL (motors, lift, lights)
    #M([F|B],[F|B],0x00,0x00)#  .. Set motors direction (F/B) and speed (HEX 0x00 .. 0xFF)
                                   M(leftDir,rightDir,leftSpeed,rightSpeed)
                                   The speed is clamped to a range min = 50, max = 150.
                                   M(F,B,0x50,0xFF) .. left motor FORWARD speed 80, right motor BACKWARD speed 255

Oba režimy lze kombinovat.

Jako odpověď posílá robot STATUS MESSAGE - data ze senzorů v pevném formátu.

    Lift(-1,1023) ODO(-8,-14) LaDis(27)
    Lift(-1,1014) ODO(-8,-14) LaDis(27)
    Lift(-1,1000) ODO(-8,-14) LaDis(27)
    Lift(-1,987) ODO(-8,-14) LaDis(27)
    Lift(-1,973) ODO(-8,-14) LaDis(27)
    Lift(-1,960) ODO(-8,-14) LaDis(32)
    Lift(-1,946) ODO(-8,-14) LaDis(32)
    Lift(-1,933) ODO(-8,-14) LaDis(29)

Lift(direction, position) .. směr pohybu 1 nahoru, 0 stojí, -1 dolů; position 0..1024 (0 dole, 1024 zvednuto)
ODO(leftODO, rightODO) .. levý a pravý tachometr
LaDis(distance) .. vzdálenost měřená laserem od zdvihu směrem dopředu, lze využít při nájezdu na balení na paletě. POZOR: Při malé vzdálenosti < 1cm dává trochu neočekávané výsledky.

Kromě status message posílá robot i komentáře k přijatým příkazům.
Např.: 

	/* Command: 'q' .. Lift DOWN */
	/* Motor speed: 50 Lift direction, position: -1, 1024 ODO & Switch: 11111100 ODO_LeftRight(-8,-14) La(ser)Dis(tance): 27 */


