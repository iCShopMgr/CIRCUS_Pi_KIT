# 更新資訊
* 2021/7/1 新增EZ Start Kit Q。
<p align="center">
  <img src="https://github.com/iCShopMgr/CIRCUS_Pi_KIT/blob/main/Album/11.png" width="700"/>
</p>

* 2021/6/22 新增MCS替代方案的Adafruit IO與Pixetto 視覺感測器。
<p align="center">
  <img src="https://github.com/iCShopMgr/CIRCUS_Pi_KIT/blob/main/Album/10.png" width="700"/>
</p>

* 2021/5/6 新增MoonCar初始化顏色感測器時可以調整比對顏色時的誤差。
* 2021/2/26 新增HUSKYLENS 與M5Stack Mini RFID Reader/Writer Unit (MFRC522)，並修正Neo Pixel Show 的Bug。
<p align="center">
  <img src="https://github.com/iCShopMgr/CIRCUS_Pi_KIT/blob/main/Album/huskylens.png" width="700"/>
</p>
<p align="center">
  <img src="https://github.com/iCShopMgr/CIRCUS_Pi_KIT/blob/main/Album/m5rfid.png" width="700"/>
</p>

* 2021/2/17 新增可自訂名稱的XBM積木，並修改PCA9685中文顯示。
<p align="center">
  <img src="https://github.com/iCShopMgr/CIRCUS_Pi_KIT/blob/main/Album/xbm.png" width="700"/>
</p>

* 2021/1/21 修正部份無法中文化問題。
* 2021/1/19 修正RockBot移動程式結合變數會錯誤的部份，以及伺服馬達函數Bug。
* 2021/1/19 新增MFRC522讀寫程式
<p align="center">
  <img src="https://github.com/iCShopMgr/CIRCUS_Pi_KIT/blob/main/Album/mfrc522_show.png" width="700"/>
</p>

* MFRC522讀寫程式使用範例。
<p align="center">
  <img src="https://github.com/iCShopMgr/CIRCUS_Pi_KIT/blob/main/Album/mfrc522_example.png" width="700"/>
</p>

* 2021/1/18 新增SGP30並修正PMS5003以及增加PMS5003T。

* 2021/1/4 新增RockBot。
<p align="center">
  <img src="https://github.com/iCShopMgr/CIRCUS_Pi_KIT/blob/main/Album/09.png" width="700"/>
</p>

* 2020/12/12 新增PCA9685。
* 2020/11/20 開啟EEPROM至視窗列表，並新增NFC/RFID讀寫功能。
* 2020/10/30 新增M5Stack官方對於M5Stick V支援的影像功能。
* 將原先使用M5Stack CORE作為核心控制M5Stick V的部份，更改為LinkIt 7697。
* 官方教學資訊的部分可以參考下列連結：
https://docs.m5stack.com/#/en/quick_start/unitv/v_function

* 本次更新使用方式可以參考Tutorial/M5StickV.pdf，範例程式則在Example/M5StickV。
<p align="center">
  <img src="https://github.com/iCShopMgr/EZ_Start_Kit_for_BlocklyDuino_feat.liou/blob/main/Album/08.png" width="700"/>
</p>

# 版本更新紀錄
* 0722 修正OLED 重複u8g2.sendBuffer();的問題.
* 0727 修正AB按鈕順序對調無法被偵測的問題.
* 0730 新增可輸入XBM自訂圖案的積木
* 1030 新增M5Stack官方對於M5Stick V支援的影像功能
* 1120 開啟EEPROM至視窗列表，並新增NFC/RFID讀寫功能。
* 1212 新增PCA9685。
* 2021/1/4 新增RockBot。
* 2021/1/18 新增SGP30並修正PMS5003以及增加PMS5003T。
* 2021/1/18 新增MFRC522讀寫程式。
* 2021/1/19 修正RockBot移動程式結合變數會錯誤的部份，以及伺服馬達函數Bug。
* 2021/1/21 修正部份無法中文化問題。
* 2021/2/17 新增可自訂名稱的XBM積木，並修改PCA9685中文顯示。
* 2021/2/26 新增HUSKYLENS 與M5Stack Mini RFID Reader/Writer Unit (MFRC522)，並修正Neo Pixel Show 的Bug。

# 自訂OLED顯示圖案教學步驟
* 開啟BlocklyDuino 並點選EZ Start Kit 可看到已擴充的程式積木，使用OLED顯示圖案會需要初始化、自訂自己的圖案、顯示等步驟，如下圖所示紅框：
<p align="center">
  <img src="https://github.com/iCShopMgr/EZ_Start_Kit_for_BlocklyDuino_feat.liou/blob/main/Album/02.png" width="700"/>
</p>

* 初始化OLED與自訂的部份我們放在初始化，顯示的部份則可以依照需求放在初始化或重複執行，在此放在重複執行，如下圖所示：
<p align="center">
  <img src="https://github.com/iCShopMgr/EZ_Start_Kit_for_BlocklyDuino_feat.liou/blob/main/Album/03.png" width="700"/>
</p>

* 接下來製作我們想顯示的圖案，用小畫家或繪圖軟體，製作一張128x64 橡素的圖片，存檔格式必須是單色點陣圖，如下圖所示：
<p align="center">
  <img src="https://github.com/iCShopMgr/EZ_Start_Kit_for_BlocklyDuino_feat.liou/blob/main/Album/04.png" width="700"/>
</p>

* 將圖片轉換成XBM格式，在此使用線上的免費服務做轉換(https://www.online-utility.org/image/convert/to/XBM)
，如下圖所示：
<p align="center">
  <img src="https://github.com/iCShopMgr/EZ_Start_Kit_for_BlocklyDuino_feat.liou/blob/main/Album/05.png" width="700"/>
</p>

* 用記事本打開轉換好的XBM檔案，將大括弧內的所有0x00, 0x....這些16進制編碼複製起來，如下圖所示：
<p align="center">
  <img src="https://github.com/iCShopMgr/EZ_Start_Kit_for_BlocklyDuino_feat.liou/blob/main/Album/06.png" width="700"/>
</p>

* 將這些編碼貼到方才我們編寫好的程式，自訂OLED點陣圖...後面的字串項內，上傳程式到LinkIt7697，即可看到OLED顯示自訂的圖案囉，如下圖所示：
<p align="center">
  <img src="https://github.com/iCShopMgr/EZ_Start_Kit_for_BlocklyDuino_feat.liou/blob/main/Album/07.png" width="700"/>
</p>

# 版本更新紀錄
* 0722 修正OLED 重複u8g2.sendBuffer();的問題.
* 0727 修正AB按鈕順序對調無法被偵測的問題.
* 0730 新增可輸入XBM自訂圖案的積木
