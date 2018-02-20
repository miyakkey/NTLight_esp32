NTLight with ESP32
====

#Overview
NTPを利用してサーバーから時刻を取得し、その時刻に応じた色を表示する照明です。  

## Description
ESP32を利用してNTPを利用してサーバーから時刻を取得します。
その時刻を画面に表示し、さらに照明の色をその時刻に合わせた色にします。  
ボタンを押すと通常の照明になります。  
~~決していつぞやらのIQLightが邪魔になったからプレゼントに使おうというのではない~~

## Requirement
* Wi-Fi アクセスポイント
* プロキシのないネットワーク  

## Usage
1. 付属のAC電源をつなぎ、電源を入れます。
2. 画面に「AP MODE」と表示されるまで待ちます。
3. 手持ちのwi-fi対応機器(ノートパソコンとかスマホとか)で「ESP-xxxxxx」を探し、接続します。
4. ブラウザでhttp://192.168.4.1にアクセスします。
5. 画面上の Access Point Name と Passward につなぎたいアクセスポイントの名前と
パスワードを入力し、Send をクリックします。
6. しばらく待ちます。時刻が画面に表示されれば成功です。

## Caution
* 接続するwi-fiの名前は平文で打電され、平文で本体に保存されます。
したがって誰かに傍受される可能性のある環境での設定および使用は避けてください。
* 画面に「AP MODE」と出ている間はアクセスポイントとして起動しています。だれでも
接続可能(パスワードなし)なのでその状態での使用は避けてください。

## interfaces
(左上から)  
* ボリューム ... 明るさの変更
* スイッチ1 ... AP MODE 起動
* スイッチ2 ... 白色点灯モード-時間の色点灯モード切替
* 画面 ... 画面(...これ以上の説明は無理)

## Install or Change the Internal Program
内部のシステムにはESP32を使っていて、main.pyが入っています。
なのでampy等のツールでシリアル越しにプログラムを書き換えることが可能です。  
1. 本体を分解します。
2. ESP32が載っているマイコンボードのmicroUSBにPCをつなぎます。  
あとはESP32にpythonを載せる方法などを参考にしてください。

## Debug
* 画面が付かない
  * 接触不良が疑われます。画面を少し押してみてください。
* スイッチ2を押したら動かなくなった
  * ESP32の性能の限界です。電源を入れなおしてください。なるべく軽く押すように心がけてください。

## Licence
プログラム...[Apache2.0](http://www.apache.org/licenses/)  
ハードウェア...要請があれば公開します。
