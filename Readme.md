# 説明

## capture
    <!-- OpenCVで画像を撮影 -->
    生をpublish(デフォ: /image_raw)
    レーザ検知用、通常用をpublish(デフォ: /image_odom, /image_laser)
## laser_binarization
    <!-- レーザ画像の二値化 -->
    レーザ検知用の画像をsubscribe(デフォ: /image_laser)
    二値化、ノイズ処理
    二値化画像をpublish(デフォ: /image_laser/binarization)
## laser_cloud
    <!-- レーザ二値化画像の3次元点群化 -->
    二値化画像をsubscribe(デフォ: /image_laser/binarization)
    カメラパラメータのインポート(デフォ: camera.yaml)
    レーザパラメータのインポート(デフォ: laser.yaml)
    点群のpublish(デフォ: /laser/points3d)(2d: /laser/points2d)
## imu_pose
    <!-- imuの自己位置推定 -->
    imu姿勢と値をsubscribe(デフォ: /imu/data)
    位置を計算(デフォ: デッドレコニング)
    tf(world -> odom)をpub
    /imu/data_odom：加速度がワールド座標系
    world -> odom：imuの位置
    odom -> imu_link：imuの角度
## imu_calibration
    imuのキャリブレーションプログラム(予定)
    現状：
        データ取得
    理想：
        加速度、ジャイロ生データ取得
            取得は全方位、散らばるようにする
        ジャイロ：バイアスを計算、差分をpub
        加速度：楕円球の真円化、パラメータでキャリブレーションしてyaml保存
## imu_calibrated
    <!-- imuデータの補正 -->
    imu.yamlに基づき/imu/data_rawを補正
    /imu/data_calibratedとしてpublish
## imu_savecsv
    imuデータをcsvファイルに保存するプログラム
    キャリブレーション用、特性チェック
## laser_binar
    二値化用コード
    背景差分のテストにつかった
    レーザのキャリブレーションのときに使った？

