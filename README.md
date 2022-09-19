<h1>Projekta darba apraksts</h1>

## Autonomā siltumnīca

Siltumnīcas plāns bija uzstaisīt, lai siltumnīca varētu regulēt gaismas režīmu, klimatu, laisīšanu, un visus sensoru datus ierakstīt.


![Siltumnīca no iekšpuses	](images/image5.jpg "Siltumnīca no iekšpuses")

![Siltumnīca no ārpuses](images/image2.jpg "Siltumnīca no ārpuses")

Siltumnīcas izmēri ir 45x45x80cm.

Siltumnīcā patreiz aug salāts, pipermēta, cūku pupas, citornzāle, pārziemota paprika.


Siltumnīcas galvenais vadības elements ir ESP32 mikrokontrolers, kam papildus ir Rasperry Pi Zero, kas uzņem foto atēlus priekš timelapse video


![ESP32](images/imag15.jpg "ESP32")

![Raspberry Pi Zero](images/image18.jpg "Raspberry Pi Zero")

![Vadības elemneti uz DIN sliedes](images/image12.jpg "Vadības elemneti uz DIN sliedes")


Visi vadi ir nenoslēgti prototipēšanas nolūkam, lai āti varētu kaut ko samainīt vai diagnosticēt.

Timelapse video 2 nedēlu periodā:

[https://youtu.be/QADKhDMOnyU](https://youtu.be/QADKhDMOnyU)

Siltumnīcā sensori:

<table>
  <tr>
   <td>
<img src="images/image11.jpg" width="" alt="AM2320" title="AM2320">

   </td>
   <td>
<img src="images/image22.jpg" width="" alt="BMP280" title="BMP280">

   </td>
  </tr>
  <tr>
   <td>AM2320
<p>
Mēra siltumnīcas iekšējo temperatūru un relatīvo gaisa mitrumu
   </td>
   <td>BMP280 mēra arējo temperatūtu un gaisa mitrumu
   </td>
  </tr>
  <tr>
   <td>
<img src="images/image1.jpg" width="" alt="Augsnes mitruma sensori" title="Augsnes mitruma sensori">

   </td>
   <td>
   </td>
  </tr>
  <tr>
   <td>Kapacatīvais un pretestības augsnes mitruma senori + augnes temperatūras sensors
   </td>
   <td>
   </td>
  </tr>
</table>


Sensorus mēģināju kalibrēt izžāvējot augsni tad un pakāpeniski to mitrinot, novēroju ekponencionālas izmaiņas starp lasījumiem, tad attiecīgi ar Python skriptu mēģināju pielāgot līknes pie datiem.


![Kapacatīvā sensora kalibrācija](images/image13.png "Kapacatīvā sensora kalibrācija")

![Rezistīvā sensora kalibrācija](images/image17.png "Rezistīvā sensora kalibrācija")



![Python skripsts](images/image3.png "Python skripsts")




Visi izpildes elementi tiek vadīti vai no bipolārlajiem vai MOSFET tranzistoriem


![alt_text](images/image6.jpg "image_tooltip")

![alt_text](images/image20.jpg "image_tooltip")


Siltumnīcas izplides elementi:


<table>
  <tr>
   <td>
<img src="images/image23.jpg" width="" alt="Ūdens sūknis" title="Ūdens sūknis">

   </td>
   <td>
<img src="images/image21.jpg" width="" alt="24V LED lenta, UV gaismas, ventilatori" title="24V LED lenta, UV gaismas, ventilatori">

   </td>
  </tr>
  <tr>
   <td>Ūdens sūknis
   </td>
   <td>24V baltās krāsas gaismas, 360nm UV gaisma, + 3 ventilatori.
   </td>
  </tr>
  <tr>
   <td>
<img src="images/image7.jpg" width="" alt="Sildelementi" title="Sildelementi">

   </td>
   <td>
<img src="images/image10.jpg" width="" alt="Gaisa mitrinātajs" title="Gaisa mitrinātajs">

   </td>
  </tr>
  <tr>
   <td>2  12v sildelementi virknē.
   </td>
   <td>Ultraskaņas atomizators - gaisa mitrinātājs
   </td>
  </tr>
</table>




Gaisa temperatūras un mitruma regulēšanas shēma:

l
![Blokshēma](images/image9.png "Blokshēma")


VPD(Vapor Pressure Deficit) ir aprēķināta vērtība no gaisa mitruma un temperatūras, kas ir labs radītājs augam vēlamiem augšanas apstakļiem relatīvi gaisa temperatūrai.

Tabula ieskatam:


![VPD tabula](images/image16.jpg "VPD tabula")


Siltumnīcas sensoru dati tiek nosūtīti caur internetu uz InfluxDB mākoņa datubāzi:
![alt_text](images/image8.png "image_tooltip")


ESP32 programma publicēta GitHub, programmēšanai izmantoju VSCode ar PlatformIO extension, ESP32 parasti programēju OTA(Over-the-air) caur WiFi tīklu:

[https://github.com/Jurgols/siltumnica32](https://github.com/Jurgols/siltumnica32/blob/master/src/main.cpp)


![VSCode](images/image4.png "VSCode")




Raspberry Pi foto ar intervālu tiek uzņeti caur Python valodā rakstītu skriptu, kas uz linux darbojās, kā servis.


![Raspberry Pi skripts](images/image14.png "Raspberry Pi skripts")
 

Pašus timelapse video veidoju ar ffmpeg programmu:
![ffmpeg komanda](images/image19.png "ffmpeg komanda")

```ffmpeg -framerate 24 -pattern_type glob -i "~/Pics/*jpg" -s:v 1920:1440 -c:v libx264 -crf 17 -pix_fmt yuv420p ~/Video/siltumnica_timelapse.mp4```


Timelapse video 1 mēneša periodā:

[https://youtu.be/B2gj5SM8boA](https://youtu.be/B2gj5SM8boA)
