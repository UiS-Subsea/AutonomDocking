# UiS Subsea - Autonomus Dokking 2024 #

### System ###

**Operativsystem:** Ubuntu 22.04 Jammy

**OpenCV**: Flg lenke for installasjon: https://opencv.org/releases/

**ROS Humble Hawksbill:** Følg lenke for installasjon og oppsett av ROS2 Humble Hawksbill: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

### Filer og mapper ###

**a_docking**: Inneholder kode for autonom docking før implementering i ROS2

**ros2_ws:** Inneholder implementert kode i ROS2

**astyring_uten_autonomi_kalibrert.py:**
Filen er ment som et supplement til manuell-kjøring. ved å kjøre denne har man bildebehandling, kalibrering og noe veiledning på bildet, der man ser i hvilken retning man bør kjøre og hvor nær eller langt unna man er.
Man kan justere på self.camera_offset_y = 50 som i dette tilfellet står for at kameraet er plassert 5 cm bak pucken man skal dokke på, dersom kameraet er plassert lenger borte fra pucken på ROV-en så justeres denne deretter.
