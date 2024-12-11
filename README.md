## mqtt_brigfe
+ ```任意のros topicをmqtt topicに変換して、AWS IoTcoreに投げる```
+ installいくつか必要、Readme_lib参照
+ build 必要 ccbp mqtt_bridge
+ CONFIG_DIRにmqtt_configというディレクトリを作ってそこに下記のconfigファイルを設置する

### config
#### params.yaml
+ ```送るトピックに変更がある時は編集必要```
#### tls_params.yaml
+ ```証明書類が変わったら変更```
#### xxxxxxxxxxxxxxxxxx-certificate.pem.crt
+ ```AWSの証明書```
#### xxxxxxxxxxxxxxxxxx-private.pem.key
+ ```AWSの証明書```
#### xxxxxxxxxxxxxxxxxx-public.pem.key
+ ```AWSの証明書```
#### AmazonRootCA1.pem
+ ```AWSの証明書```
