# Onethinx



In dit project wordt een onethinx module geprogrammeerd met een gemodificeerde psoc 6 wifi-bt
De onethinx module leest analoge data uit van een sensor die de water diepte meet en stuurt deze waarde naar de lora gateway.
In de gateway webinterface kan de ontvangen data gezien worden.

![PXL_20221230_223926812](https://user-images.githubusercontent.com/69217508/210148726-828cacad-d41c-4acf-9ffa-9deecfd0d892.jpg)


sensor:
![PXL_20221231_154327934](https://user-images.githubusercontent.com/69217508/210148819-5375de14-35e9-4b9a-88a5-c1856e9bc16a.jpg)



om psoc wifi-bt  als programmer te gebruiken:
desoldeer de volgende weerstanden
![PXL_20221231_152647527](https://user-images.githubusercontent.com/69217508/210142503-422bd518-a473-4423-9646-8b9f67ccd8b5.jpg)

verwijder plastic van plastic connector j11 en verbind probes: 
![PXL_20221231_152559205](https://user-images.githubusercontent.com/69217508/210142771-b293ada1-bbe3-4537-804e-e8725bb797e7.jpg)

verbind deze met de onethinx
![PXL_20221231_152604138](https://user-images.githubusercontent.com/69217508/210148666-b6369de5-fbd1-4f8f-b1c7-5ccb674a9b21.jpg)


## NOTE
in de device configurator staat dat CLK_PERI 16 MHz is deze is echter 50 MHz.
Als deze clock gebruikt wordt moet er de clock divider maal 3.125 moet worden gedaan om de juiste frequentie te bekomen.
![image](https://user-images.githubusercontent.com/69217508/210148976-824a3551-34e4-4a34-b2e7-45293fa17db6.png)

#### bij UART gaat dit niet door een constraint. Dus de baud rate van UART moet 360 000 zijn voor seriele data uit te kunnen lezen
![image](https://user-images.githubusercontent.com/69217508/210149052-29192e18-6f5f-4717-a90d-b48bd1936197.png)


