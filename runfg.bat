C:
cd C:\Program Files\FlightGear

SET FG_ROOT=C:\Program Files\FlightGear\data
SET FG_SCENERY=C:\Program Files\FlightGear\data\Scenery;C:\Program Files\FlightGear\scenery;C:\Program Files\FlightGear\terrasync
.\\bin\win64\fgfs --aircraft=HL21 --fdm=null --enable-auto-coordination --native-fdm=socket,in,30,localhost,5502,udp --fog-nicest --enable-clouds3d --enable-textures --enable-hud --start-date-lat=2004:06:01:09:00:00 --enable-sound --visibility=47000 --in-air --prop:/engines/engine0/running=true --disable-freeze --airport=KTTS --runway=15 --altitude=1000 --heading=0 --offset-distance=0 --offset-azimuth=0 --enable-rembrandt 
