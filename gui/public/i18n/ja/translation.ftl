### SlimeVR complete GUI translations
# Please developers (not translators) don't reuse a key inside another key
# or concat text with a translation string in the code, use the appropriate
# features like variables and selectors in each appropriate case!
# And also comment the string if it's something not easy to translate, so you help
# translators on what it means

## Websocket (server) status
websocket-connecting = サーバー接続中
websocket-connection_lost = サーバーへの接続が失われました。再接続を試みています...

## Tips
tips-find_tracker = どのトラッカーがどれだかわからない？トラッカーを振ると、該当する項目がハイライトされます。
tips-do_not_move_heels = レコーディング中にかかとが動かないように注意しましょう！

## Body parts
body_part-NONE = 未設定
body_part-HEAD = 頭
body_part-NECK = 首
body_part-RIGHT_SHOULDER = 右肩
body_part-RIGHT_UPPER_ARM = 右上腕
body_part-RIGHT_LOWER_ARM = 右前腕
body_part-RIGHT_HAND = 右手
body_part-RIGHT_UPPER_LEG = 右膝
body_part-RIGHT_LOWER_LEG = 右足
body_part-RIGHT_FOOT = 右足先
body_part-RIGHT_CONTROLLER = 右コントローラ
body_part-CHEST = 胸
body_part-WAIST = 腰
body_part-HIP = ヒップ
body_part-LEFT_SHOULDER = 左肩
body_part-LEFT_UPPER_ARM = 左上腕
body_part-LEFT_LOWER_ARM = 左前腕
body_part-LEFT_HAND = 左手
body_part-LEFT_UPPER_LEG = 左膝
body_part-LEFT_LOWER_LEG = 左足
body_part-LEFT_FOOT = 左足先
body_part-LEFT_CONTROLLER = 左コントローラ

## Skeleton stuff
skeleton_bone-NONE = 無し
skeleton_bone-HEAD = ヘッドシフト
skeleton_bone-NECK = 首長さ
skeleton_bone-TORSO = 胴長さ
skeleton_bone-CHEST = 胸部の距離
skeleton_bone-WAIST = 腰部の距離
skeleton_bone-HIP_OFFSET = ヒップオフセット
skeleton_bone-HIPS_WIDTH = ヒップ幅
skeleton_bone-LEGS_LENGTH = 脚長さ
skeleton_bone-KNEE_HEIGHT = 膝高さ
skeleton_bone-FOOT_LENGTH = 足先長さ
skeleton_bone-FOOT_SHIFT = 足先シフト
skeleton_bone-SKELETON_OFFSET = スケルトンオフセット
skeleton_bone-CONTROLLER_DISTANCE_Z = コントローラ距離 Z
skeleton_bone-CONTROLLER_DISTANCE_Y = コントローラ距離 Y
skeleton_bone-FOREARM_LENGTH = 前腕長さ
skeleton_bone-SHOULDERS_DISTANCE = 肩の距離
skeleton_bone-SHOULDERS_WIDTH = 肩幅
skeleton_bone-UPPER_ARM_LENGTH = 上腕長さ
skeleton_bone-ELBOW_OFFSET = 肘オフセット

## Tracker reset buttons
reset-reset_all = すべてのプロポーションをリセット
reset-full = リセット
reset-mounting = リセットマウンティング
reset-quick = クイックリセット

## Serial detection stuff
serial_detection-new_device-p0 = 新しいシリアルデバイスを検出しました！
serial_detection-new_device-p1 = wifiの認証情報を入力してください！
serial_detection-new_device-p2 = 何をするか選択してください
serial_detection-open_wifi = WiFiに接続
serial_detection-open_serial = シリアルコンソールを開く
serial_detection-submit = 実行！
serial_detection-close = 閉じる

## Navigation bar
navbar-home = ホーム
navbar-body_proportions = ボディプロポーション
navbar-trackers_assign = トラッカー割り当て
navbar-mounting = マウントキャリブレーション
navbar-onboarding = セットアップ ウィザード
navbar-settings = 設定

## Bounding volume hierarchy recording
bvh-start_recording = BVHレコーディング
bvh-recording = レコーディング中...

## Overlay settings
overlay-is_visible_label = SteamVRでオーバーレイを表示する
overlay-is_mirrored_label = オーバーレイをミラーとして表示する

## Tracker status
tracker-status-none = ステータスなし
tracker-status-busy = Busy
tracker-status-error = エラー
tracker-status-disconnected = 切断
tracker-status-occluded = Occluded
tracker-status-ok = 接続中

## Tracker status columns
tracker-table-column-name = Name
tracker-table-column-type = Type
tracker-table-column-battery = Battery
tracker-table-column-ping = Ping
tracker-table-column-rotation = Rotation X/Y/Z
tracker-table-column-position = Position X/Y/Z
tracker-table-column-url = URL

## Tracker rotation
tracker-rotation-front = 前
tracker-rotation-left = 左
tracker-rotation-right = 右
tracker-rotation-back = 後

## Tracker information
tracker-infos-manufacturer = メーカ－
tracker-infos-display_name = 表示名
tracker-infos-custom_name = カスタム名称
tracker-infos-url = トラッカーURL

## Tracker settings
tracker-settings-back = トラッカーリストへ戻る
tracker-settings-title = トラッカー設定
tracker-settings-assignment_section = 割り当て
tracker-settings-assignment_section-description = トラッカーが体のどの部位に装着されているか
tracker-settings-assignment_section-edit = 割り当ての編集
tracker-settings-mounting_section = 装着方向
tracker-settings-mounting_section-description = トラッカーをどの方向に装着していますか?
tracker-settings-mounting_section-edit = 装着向きの編集
tracker-settings-drift_compensation_section = Allow drift compensation
tracker-settings-drift_compensation_section-description = Should this tracker compensate for its drift when drift compensation is enabled?
tracker-settings-drift_compensation_section-edit = Allow drift compensation
# The .<name> means it's an attribute and it's related to the top key.
# In this case that is the settings for the assignment section.
tracker-settings-name_section = トラッカー名称
tracker-settings-name_section-description = 自由に名称をつけてください
tracker-settings-name_section-placeholder = NightyBeast's left leg

## Tracker part card info
tracker-part_card-no_name = 名称無し
tracker-part_card-unassigned = 未割り当て

## Body assignment menu
body_assignment_menu = このトラッカーをどこに配置しますか？
body_assignment_menu-description = このトラッカーを割り当てる場所を選択します。また、トラッカーを一つずつ管理するのではなく、すべてのトラッカーを一括して管理することもできます。
body_assignment_menu-show_advanced_locations = 高度な割り当て場所の表示
body_assignment_menu-manage_trackers = すべてのトラッカーの管理
body_assignment_menu-unassign_tracker = トラッカーの割り当て解除

## Tracker assignment menu
# A -translation_key (with a dash in the front) means that it's a label.
# It can only be used in the translation file, it's nice for reusing names and that kind of stuff.
# 
# We are using it here because english doesn't require changing the text in each case but
# maybe your language does.
-tracker_selection-part = Which tracker to assign to your
tracker_selection_menu-NONE = Which tracker do you want to be unassigned?
tracker_selection_menu-HEAD = { -tracker_selection-part } head?
tracker_selection_menu-NECK = { -tracker_selection-part } neck?
tracker_selection_menu-RIGHT_SHOULDER = { -tracker_selection-part } right shoulder?
tracker_selection_menu-RIGHT_UPPER_ARM = { -tracker_selection-part } right upper arm?
tracker_selection_menu-RIGHT_LOWER_ARM = { -tracker_selection-part } right lower arm?
tracker_selection_menu-RIGHT_HAND = { -tracker_selection-part } right hand?
tracker_selection_menu-RIGHT_UPPER_LEG = { -tracker_selection-part } right thigh?
tracker_selection_menu-RIGHT_LOWER_LEG = { -tracker_selection-part } right ankle?
tracker_selection_menu-RIGHT_FOOT = { -tracker_selection-part } right foot?
tracker_selection_menu-RIGHT_CONTROLLER = { -tracker_selection-part } right controller?
tracker_selection_menu-CHEST = { -tracker_selection-part } chest?
tracker_selection_menu-WAIST = { -tracker_selection-part } waist?
tracker_selection_menu-HIP = { -tracker_selection-part } hip?
tracker_selection_menu-LEFT_SHOULDER = { -tracker_selection-part } left shoulder?
tracker_selection_menu-LEFT_UPPER_ARM = { -tracker_selection-part } left upper arm?
tracker_selection_menu-LEFT_LOWER_ARM = { -tracker_selection-part } left lower arm?
tracker_selection_menu-LEFT_HAND = { -tracker_selection-part } left hand?
tracker_selection_menu-LEFT_UPPER_LEG = { -tracker_selection-part } left thigh?
tracker_selection_menu-LEFT_LOWER_LEG = { -tracker_selection-part } left ankle?
tracker_selection_menu-LEFT_FOOT = { -tracker_selection-part } left foot?
tracker_selection_menu-LEFT_CONTROLLER = { -tracker_selection-part } left controller?

tracker_selection_menu-unassigned = 未割り当てのトラッカー
tracker_selection_menu-assigned = 割り当て済みのトラッカー
tracker_selection_menu-dont_assign = 割り当てない

## Mounting menu
mounting_selection_menu = このトラッカーをどこに配置しますか？
mounting_selection_menu-close = 閉じる

## Sidebar settings
settings-sidebar-title = 設定
settings-sidebar-general = 一般
settings-sidebar-tracker_mechanics = トラッカーメカニズム
settings-sidebar-fk_settings = FK設定
settings-sidebar-gesture_control = Gesture control
settings-sidebar-interface = インターフェース
settings-sidebar-osc_router = OSC router
settings-sidebar-utils = ユーティリティ
settings-sidebar-serial = シリアルコンソール

## SteamVR settings
settings-general-steamvr = SteamVR
settings-general-steamvr-subtitle = SteamVRのトラッカー
# Not all translation keys support multiline, only the ones that specify it will actually
# split it in lines (that also means you can split in lines however you want in those).
# The first spaces (not tabs) for indentation will be ignored, just to make the file look nice when writing.
# This one is one of this cases that cares about multilines
settings-general-steamvr-description =
    有効化したい部位にチャックを入れてください。
    SlimeVRが行うことをよりコントロールしたい場合に便利です。
settings-general-steamvr-trackers-waist = 腰
settings-general-steamvr-trackers-chest = 胸
settings-general-steamvr-trackers-feet = 足
settings-general-steamvr-trackers-knees = 膝
settings-general-steamvr-trackers-elbows = 肘
settings-general-steamvr-trackers-hands = Hands

## Tracker mechanics
settings-general-tracker_mechanics = トラッカーメカニズム
settings-general-tracker_mechanics-filtering = フィルター機能
# This also cares about multilines
settings-general-tracker_mechanics-filtering-description =
    トラッカーのフィルタリングの種類を選択します。
    Predictionは動きを予測し、Smoothingは動きを滑らかにする。
settings-general-tracker_mechanics-filtering-type = フィルタータイプ
settings-general-tracker_mechanics-filtering-type-none = No filtering
settings-general-tracker_mechanics-filtering-type-none-description = トラッカーの値をそのまま使用します。フィルタリングは行いません。
settings-general-tracker_mechanics-filtering-type-smoothing = Smoothing
settings-general-tracker_mechanics-filtering-type-smoothing-description = 動きを滑らかにしますが、若干の遅れが発生します
settings-general-tracker_mechanics-filtering-type-prediction = Prediction
settings-general-tracker_mechanics-filtering-type-prediction-description = レイテンシーを減らし、動きをよりキビキビさせますが、ジッターが増加する場合があります。
settings-general-tracker_mechanics-filtering-amount = 数値
settings-general-tracker_mechanics-drift_compensation = Drift compensation
# This cares about multilines
settings-general-tracker_mechanics-drift_compensation-description =
    Compensates IMU yaw drift by applying an inverse rotation.
    Change amount of compensation and up to how many resets are taken into account.
settings-general-tracker_mechanics-drift_compensation-enabled-label = Drift compensation
settings-general-tracker_mechanics-drift_compensation-amount-label = Compensation amount
settings-general-tracker_mechanics-drift_compensation-max_resets-label = Use up to x last resets

## FK settings
settings-general-fk_settings = FK設定
settings-general-fk_settings-leg_tweak = 脚の微調整
settings-general-fk_settings-leg_tweak-description = フロアクリップは、床とのクリッピングを減らす、あるいはなくすことができますが、膝をついたときに問題が発生する可能性があります。スケーティング補正は足の滑りを補正できますが、特定の動作パターンでは精度が落ちることがあります。
# Floor clip: 
# why the name - came from the idea of noclip in video games, but is the opposite where clipping to the floor is a desired feature
# definition - Prevents the foot trackers from going lower than they where when a reset was performed
settings-general-fk_settings-leg_tweak-floor_clip = フロアクリップ
# Skating correction: 
# why the name - without this enabled the feet will often slide across the ground as if your skating across the ground,
# since this largely prevents this it corrects for it hence skating correction (note this may be renamed to sliding correction)
# definition - Guesses when each foot is in contact with the ground and uses that information to improve tracking
settings-general-fk_settings-leg_tweak-skating_correction = スケーティング補正
settings-general-fk_settings-leg_tweak-skating_correction-amount = Skating correction strength
settings-general-fk_settings-arm_fk = アームFK
settings-general-fk_settings-arm_fk-description = 腕の追従方法を変更する。
settings-general-fk_settings-arm_fk-force_arms = Force arms from HMD
settings-general-fk_settings-skeleton_settings = スケルトン設定
settings-general-fk_settings-skeleton_settings-description = スケルトン設定のオン/オフを切り替えます。これらはオンのままにしておくことをお勧めします。
settings-general-fk_settings-skeleton_settings-extended_spine = Extended spine
settings-general-fk_settings-skeleton_settings-extended_pelvis = Extended pelvis
settings-general-fk_settings-skeleton_settings-extended_knees = Extended knee
settings-general-fk_settings-vive_emulation-title = Vive emulation
settings-general-fk_settings-vive_emulation-description = Emulate the waist tracker problems that Vive trackers have. This is a joke and makes tracking worse.
settings-general-fk_settings-vive_emulation-label = Enable Vive emulation

## Gesture control settings (tracker tapping)
settings-general-gesture_control = ジェスチャーコントロール
settings-general-gesture_control-subtitle = ダブルタップクイックリセット
settings-general-gesture_control-description = ダブルタップクイックリセットの有効・無効を設定します。有効にすると、最も高い胴体トラッカー上の任意の場所をダブルタップすると、クイックリセットが起動します。ディレイは、タップされてからリセットされるまでの時間です。
# This is a unit: 3 taps, 2 taps, 1 tap
# $amount (Number) - Amount of taps (touches to the tracker's case)
settings-general-gesture_control-taps = { $amount ->
    [one] 1 tap
    *[other] { $amount } taps
}
settings-general-gesture_control-quickResetEnabled = Enable tap to quick reset
settings-general-gesture_control-quickResetDelay = Quick reset delay
settings-general-gesture_control-quickResetTaps = Taps for quick reset
settings-general-gesture_control-resetEnabled = Enable tap to reset
settings-general-gesture_control-resetDelay = Reset delay
settings-general-gesture_control-resetTaps = Taps for reset
settings-general-gesture_control-mountingResetEnabled = Enable tap to reset mounting
settings-general-gesture_control-mountingResetDelay = Mounting reset delay
settings-general-gesture_control-mountingResetTaps = Taps for mounting reset

## Interface settings
settings-general-interface = インターフェース
settings-general-interface-dev_mode = 開発者モード
settings-general-interface-dev_mode-description = このモードは、詳細なデータが必要な場合や、接続されたトラッカーをより高度なレベルで操作する場合に役立ちます。
settings-general-interface-dev_mode-label = 開発者モード
settings-general-interface-serial_detection = シリアルデバイスの検出
settings-general-interface-serial_detection-description = このオプションは、トラッカーとなり得る新しいシリアルデバイスを接続するたびにポップアップを表示します。これはトラッカーの設定プロセスを改善するのに役立ちます。
settings-general-interface-serial_detection-label = シリアルデバイスの検出
settings-general-interface-lang = 言語を選択
settings-general-interface-lang-description = 使用したいデフォルトの言語を変更する
settings-general-interface-lang-placeholder = 使用する言語を選択する

## Serial settings
settings-serial = シリアルコンソール
# This cares about multilines
settings-serial-description =
    シリアル通信のライブ情報フィードです。
    ファームウェアの動作を知る必要がある場合に有用かもしれません。
settings-serial-connection_lost = シリアルへの接続が失われました、再接続中...
settings-serial-reboot = リブート
settings-serial-factory_reset = ファクトリーリセット
settings-serial-get_infos = 情報取得
settings-serial-serial_select = シリアルポートを選択
settings-serial-auto_dropdown_item = 自動

## OSC router settings
settings-osc-router = OSC router
# This cares about multilines
settings-osc-router-description =
    Forward OSC messages from another program.
    Useful for using another OSC program with VRChat for example.
settings-osc-router-enable = Enable
settings-osc-router-enable-description = Toggle the forwarding of messages.
settings-osc-router-enable-label = Enable
settings-osc-router-network = Network ports
# This cares about multilines
settings-osc-router-network-description =
    Set the ports for listening and sending data.
    These can be the same as other ports used in the SlimeVR server.
settings-osc-router-network-port_in =
    .label = Port In
    .placeholder = Port in (default: 9002)
settings-osc-router-network-port_out =
    .label = Port Out
    .placeholder = Port out (default: 9000)
settings-osc-router-network-address = Network address
settings-osc-router-network-address-description = Set the address to send out data at.
settings-osc-router-network-address-placeholder = IPV4 address

## OSC VRChat settings
settings-osc-vrchat = VRChat OSC Trackers
# This cares about multilines
settings-osc-vrchat-description =
    HMDのデータを受信して送信するためにVRChat固有の設定を変更する。
    FBT用のトラッカーデータ（Questスタンドアロンで動作します）
settings-osc-vrchat-enable = 有効
settings-osc-vrchat-enable-description = データの送受信を切り替える。
settings-osc-vrchat-enable-label = 有効
settings-osc-vrchat-network = ネットワークポート
settings-osc-vrchat-network-description = VRChatへのデータを送受信するためのポートを設定します。
settings-osc-vrchat-network-port_in =
    .label = ポートイン
    .placeholder = ポートイン (デフォルト: 9001)
settings-osc-vrchat-network-port_out =
    .label = ポートアウト
    .placeholder = ポートアウト (デフォルト: 9000)
settings-osc-vrchat-network-address = ネットワークアドレス
settings-osc-vrchat-network-address-description = VRChatにデータを送信するアドレスを選択してください（デバイスのwifi設定を確認してください）
settings-osc-vrchat-network-address-placeholder = VRChatのIPアドレス
settings-osc-vrchat-network-trackers = トラッカー
settings-osc-vrchat-network-trackers-description = データの送受信を切り替える。
settings-osc-vrchat-network-trackers-chest = 胸
settings-osc-vrchat-network-trackers-waist = 腰
settings-osc-vrchat-network-trackers-knees = 膝
settings-osc-vrchat-network-trackers-feet = 足
settings-osc-vrchat-network-trackers-elbows = 肘

## Setup/onboarding menu
onboarding-skip = 設定をスキップする
onboarding-continue = 続ける
onboarding-wip = 実行中

## WiFi setup
onboarding-wifi_creds-back = 戻る
onboarding-wifi_creds = WiFiの認証情報の入力
# This cares about multilines
onboarding-wifi_creds-description =
    トラッカーはこれらの認証情報を使ってwifiに接続します。
    現在接続している認証情報を使用してください。
onboarding-wifi_creds-skip = wifi設定をスキップする
onboarding-wifi_creds-submit = 実行！
onboarding-wifi_creds-ssid =
    .label = SSID
    .placeholder = Enter SSID
onboarding-wifi_creds-password =
    .label = Password
    .placeholder = Enter password

## Mounting setup
onboarding-reset_tutorial-back = マウントキャリブレーションに戻る
onboarding-reset_tutorial = リセットチュートリアル
onboarding-reset_tutorial-description = この機能は終了していません。続けるを押してください。

## Setup start
onboarding-home = Welcome to SlimeVR
# This cares about multilines and it's centered!!
onboarding-home-description =
    Bringing full-body tracking
    to everyone
onboarding-home-start = セットアップ開始！

## Enter VR part of setup
onboarding-enter_vr-back = トラッカー割り当てに戻る
onboarding-enter_vr-title = VRに入る時間だ！
onboarding-enter_vr-description = トラッカーを全部つけて、VRに突入せよ！
onboarding-enter_vr-ready = 準備完了

## Setup done
onboarding-done-title = 準備完了です！
onboarding-done-description = フルトラをお楽しみください
onboarding-done-close = ガイドを閉じる

## Tracker connection setup
onboarding-connect_tracker-back = WiFi認証に戻る
onboarding-connect_tracker-title = 接続中のトラッカー
onboarding-connect_tracker-description-p0 = さあ、楽しい部分に移りましょう。すべてのトラッカーを接続します！
onboarding-connect_tracker-description-p1 = まだ接続されていないトラッカーたちをUSBポートを通して接続するだけです。
onboarding-connect_tracker-issue-serial = 接続に問題があります！
onboarding-connect_tracker-usb = USBトラッカー
onboarding-connect_tracker-connection_status-connecting = wifiの認証情報を送信中
onboarding-connect_tracker-connection_status-connected = WiFiに接続されました
onboarding-connect_tracker-connection_status-error = Wifiに接続できません
onboarding-connect_tracker-connection_status-start_connecting = トラッカーを探しています
onboarding-connect_tracker-connection_status-handshake = サーバーに接続されました
# $amount (Number) - Amount of trackers connected (this is a number, but you can use CLDR plural rules for your language)
# More info on https://www.unicode.org/cldr/cldr-aux/charts/22/supplemental/language_plural_rules.html
# English in this case only has 2 plural rules, which are "one" and "other",
# we use 0 in an explicit way because there is no plural rule in english for 0, so we directly say
# if $amount is 0 then we say "No trackers connected"
onboarding-connect_tracker-connected_trackers = { $amount ->
    [0] No trackers
    [one] 1 tracker
    *[other] { $amount } trackers
} connected
onboarding-connect_tracker-next = すべてのトラッカーを接続しました

## Tracker assignment setup
onboarding-assign_trackers-back = Wifi認証に戻る
onboarding-assign_trackers-title = トラッカーを割り当てる
onboarding-assign_trackers-description = どのトラッカーをどこに置くか選んでみましょう。トラッカーを配置したい場所をクリックしてください。
# Look at translation of onboarding-connect_tracker-connected_trackers on how to use plurals
# $assigned (Number) - Trackers that have been assigned a body part
# $trackers (Number) - Trackers connected to the server
onboarding-assign_trackers-assigned = { $assigned } of { $trackers ->
    [one] 1 tracker
    *[other] { $trackers } trackers
} assigned
onboarding-assign_trackers-advanced = 高度な割り当て場所の表示
onboarding-assign_trackers-next = すべてのトラッカーを割り当てました

## Tracker manual mounting setup
onboarding-manual_mounting-back = VRの入力に戻る
onboarding-manual_mounting = マニュアルマウント
onboarding-manual_mounting-description = すべてのトラッカーをクリックし、どの方向にマウントするかを選択
onboarding-manual_mounting-auto_mounting = 自動マウント
onboarding-manual_mounting-next = 次のステップ

## Tracker automatic mounting setup
onboarding-automatic_mounting-back = VRの入力に戻る
onboarding-automatic_mounting-title = マウントキャリブレーション
onboarding-automatic_mounting-description = SlimeVRのトラッカーを動作させるためには、物理的なトラッカーの取り付け位置と合わせるために、トラッカーの取り付け方向を合わせる必要があります。
onboarding-automatic_mounting-manual_mounting = マニュアルマウント
onboarding-automatic_mounting-next = 次のステップ
onboarding-automatic_mounting-prev_step = 前のステップ
onboarding-automatic_mounting-done-title = 取り付け方向の較正を行いました。
onboarding-automatic_mounting-done-description = マウントのキャリブレーションが完了しました！
onboarding-automatic_mounting-done-restart = 最初に戻る
onboarding-automatic_mounting-mounting_reset-title = マウントリセット
onboarding-automatic_mounting-mounting_reset-step-0 = 1. 足を曲げ、上体を前に倒し、腕を曲げた状態で、スキーのポーズでしゃがむ。
onboarding-automatic_mounting-mounting_reset-step-1 = 2. リセットマウンティングボタンを押し、3秒待つと装着方向がリセットされます。
onboarding-automatic_mounting-preparation-title = 準備
onboarding-automatic_mounting-preparation-step-0 = 1. 両手を横に広げて直立します。
onboarding-automatic_mounting-preparation-step-1 = 2. リセットボタンを押し、3秒待つとリセットされます。
onboarding-automatic_mounting-put_trackers_on-title = トラッカーを装着する
onboarding-automatic_mounting-put_trackers_on-description = マウントの方向を較正するために、先ほど割り当てたトラッカーを使用します。右の図でどれがどれだかわかると思います。
onboarding-automatic_mounting-put_trackers_on-next = すべてのトラッカーを装着しました

## Tracker manual proportions setup
onboarding-manual_proportions-back = チュートリアルをリセットする
onboarding-manual_proportions-title = マニュアルボディプロポーション
onboarding-manual_proportions-precision = 精度を調整する
onboarding-manual_proportions-auto = 自動キャリブレーション

## Tracker automatic proportions setup
onboarding-automatic_proportions-back = チュートリアルをリセットする
onboarding-automatic_proportions-title = 自分の体の測定
onboarding-automatic_proportions-description = SlimeVRのトラッカーが動作するためには、あなたの骨の長さを知る必要があります。この短いキャリブレーションでそれを測定します。
onboarding-automatic_proportions-manual = 手動調整
onboarding-automatic_proportions-prev_step = 前のステップ
onboarding-automatic_proportions-put_trackers_on-title = トラッカーを装着する
onboarding-automatic_proportions-put_trackers_on-description = プロポーションを調整するために、先ほど割り当てたトラッカーを使用します。右の図で、どれがどのトラッカーかわかると思います。
onboarding-automatic_proportions-put_trackers_on-next = すべてのトラッカーを装着しました
onboarding-automatic_proportions-preparation-title = 準備
onboarding-automatic_proportions-preparation-description = プレイスペース内のあなたの真後ろに椅子を置いてください。オートボーンセットアップの間、座れるように準備してください。
onboarding-automatic_proportions-preparation-next = 椅子の前にいます
onboarding-automatic_proportions-start_recording-title = 測定の準備をする
onboarding-automatic_proportions-start_recording-description = これから具体的なポーズや動きを記録します。これらは次の画面に表示されます。ボタンが押されたらすぐに始められるように準備しておいてください！
onboarding-automatic_proportions-start_recording-next = レコーディングスタート
onboarding-automatic_proportions-recording-title = REC
onboarding-automatic_proportions-recording-description-p0 = レコーディング中...
onboarding-automatic_proportions-recording-description-p1 = 以下に示すような動きをします。
onboarding-automatic_proportions-recording-steps-0 = 膝を数回曲げてください
onboarding-automatic_proportions-recording-steps-1 = 椅子に座り、立ち上がる
onboarding-automatic_proportions-recording-steps-2 = 上半身を左にひねり、右に曲げる
onboarding-automatic_proportions-recording-steps-3 = 上半身を右にひねり、左に曲げる
onboarding-automatic_proportions-recording-steps-4 = タイマーが終わるまで体をくねらせる
onboarding-automatic_proportions-recording-processing = 結果を処理中
# $time (Number) - Seconds left for the automatic calibration recording to finish (max 15)
onboarding-automatic_proportions-recording-timer = { $time ->
    [one] 1 second left
    *[other] { $time } seconds left
}
onboarding-automatic_proportions-verify_results-title = 結果を確認
onboarding-automatic_proportions-verify_results-description = 以下の結果を確認してください。正しく表示されていますか？
onboarding-automatic_proportions-verify_results-results = 記録結果
onboarding-automatic_proportions-verify_results-processing = 結果の処理
onboarding-automatic_proportions-verify_results-redo = レコーディングやり直し
onboarding-automatic_proportions-verify_results-confirm = 正確です
onboarding-automatic_proportions-done-title = 体を測定して保存
onboarding-automatic_proportions-done-description = ボディプロポーションのキャリブレーションが完了しました！

## Home
home-no_trackers = トラッカーを検出できません。もしくは割り当てられていません。
