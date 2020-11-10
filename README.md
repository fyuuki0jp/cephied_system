# cephied_system : 深層学習を用いて室内環境光を利用した自己位置推定を行う

- 自己位置推定の手順
  1. ADコンバータを用いて50kspsで光センサから環境光を取得する
  1. 取得した環境光データをフーリエ変換し複素数の周波数スペクトルを獲得する
  1. 複素数の周波数スペクトルを学習した深層学習モデルに入力し推論する
  1. 推論結果を出力する
  
- 現在の進捗
  - [x] データセットの作成（クラス数4)
  - [x] 学習モデルの最適化
  - [x] 実装
  - [ ] テスト
  - [ ] データセットの拡張（クラス数20)
  - [ ] 学習
  - [ ] 実装 テスト
  
