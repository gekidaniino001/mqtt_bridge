## mqtt_brigfe
+ ```任意のros topicをmqtt topicに変換して、AWS IoTcoreに投げる```
+ installいくつか必要、Readme_lib参照
+ build 必要 ccbp mqtt_bridge
+ 下記のconfigファイルを設置必要、driveのautoware/universeにある。
+ pkgディレクトリの直下にconfigというディレクトリを作って6個いれておく。
+ demo.launch.pyを起動 (環境、機体ごとにconfigファイルが必要、pkg直下のconfigディレクトリを作成して置く。) 

### config
#### params.yaml
+ ```送るトピックに変更がある時は編集必要```
#### tls_params.yaml
+ ```証明書類が変わったら変更```
#### xxxxxxxxxxxxxxxxxx-certificate.pem.crt
+ ```Ksysにもらう```
#### xxxxxxxxxxxxxxxxxx-private.pem.key
+ ```Ksysにもらう```
#### xxxxxxxxxxxxxxxxxx-public.pem.key
+ ```Ksysにもらう```
#### AmazonRootCA1.pem
+ ```Ksysにもらう```
