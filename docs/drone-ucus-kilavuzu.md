# Helsinki ortamında drone uçuşum

Bu sürümü Apple Silicon macOS, Gazebo Harmonic 8.14.0, Metal ve Python 3.10.18
ile çalıştırdım. Buradaki komutlar yalnız yerel Gazebo + ArduPilot SITL içindir;
gerçek drone veya seri port bağlantısı kullanmıyorum. GPS açık.
Linux/Windows desteğini bu sürümde doğrulamadım.

## Kurulum

Önce Gazebo Harmonic'i ve Xcode Command Line Tools'u kurup `gz` komutunun
çalıştığını kontrol ediyorum. [Gazebo–ArduPilot kurulum belgesi](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)
ve [macOS geliştirme ortamı](https://ardupilot.org/dev/docs/building-setup-mac.html)
bağımlılıklar için başvurduğum kaynaklar.

Depo kökünde Python ortamımı hazırlıyorum:

```bash
python3.10 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements-drone.txt
python3 scripts/download_gazebo_assets.py
```

Gazebo kurulu olduktan sonra eklenti ve koordinat dönüşümü bağımlılıklarını kuruyorum:

```bash
brew install cmake pkg-config proj rapidjson opencv gstreamer
```

ArduPilot ve Gazebo eklentisini aşağıdaki sabit sürümlerden derledim.
Bu komutları yeni kurulumda kullanıyorum; mevcut bir `vendor` içeriğini
silip üzerine yazmıyorum. İlk derleme ve alt modül indirmeleri zaman alabilir.

```bash
mkdir -p vendor
git clone --no-checkout https://github.com/ArduPilot/ardupilot.git vendor/ardupilot
git -C vendor/ardupilot checkout 2d534eb84267867f66e12c87f256af6ed7af33de
git -C vendor/ardupilot submodule update --init --recursive
```

ArduPilot kaynak dizininde, aynı Python ortamı açıkken:

```bash
cd vendor/ardupilot
python3 modules/waf/waf-light configure --board sitl --no-submodule-update
python3 modules/waf/waf-light copter
cd ../..
```

Gazebo eklentisini depo kökünden hazırlıyorum:

```bash
git clone --no-checkout https://github.com/ArduPilot/ardupilot_gazebo.git vendor/ardupilot_gazebo
git -C vendor/ardupilot_gazebo checkout 082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5
GZ_VERSION=harmonic cmake -S vendor/ardupilot_gazebo -B build/ardupilot_gazebo -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build/ardupilot_gazebo --target ArduPilotPlugin -j 4
```

Beklediğim iki derleme çıktısı:

- `vendor/ardupilot/build/sitl/bin/arducopter`
- `build/ardupilot_gazebo/libArduPilotPlugin.dylib`

Kaynak sürümlerini [drone-sources.json](../config/drone-sources.json) dosyasına da
yazdım. Başlatıcı eklenti ve model yollarını yalnız kendi alt süreçlerine
veriyor; `.zshrc` veya sistem geneli ayarları değiştirmiyor.

## Açma ve uçurma

Depo kökünde, Python ortamı açıkken:

```bash
python3 scripts/fly_drone.py
```

Önce yalnız dünya ve yerel modelleri kontrol etmek için `--check` kullanıyorum.
Başlatıcı macOS'un ayrı sunucu/arayüz gereksinimini kendi iki alt süreciyle
karşılıyor; ikinci terminal açmam gerekmiyor.

`Hazır` yazısından sonra aynı terminalde her `hover` mesajını bekleyerek:

```text
kalkis
git 5 0
git 5 4
don
```

`git` hedeflerini ilk Home'a göre kuzey/doğu metreleriyle veriyorum. `in`
bulunduğu yerde iniş istiyor; `don` ilk H noktasına dönüp iniyor. `durum`
Home'a uzaklığı gösteriyor. `landed` mesajını gördükten sonra `cik` ile
yalnız bu oturumu kapatıyorum. Havada `cik` vermeyi Home'a dönüş komutu
olarak kullanmıyorum.

İlk hedefler Home'un 10 m çevresiyle sınırlı. Seçtiğim ilk yükseklik yaklaşık
18 m. Başka yerde inip tekrar kalkınca Home'u değiştirmiyorum; yeniden
kalkış bulunduğu xy üzerinde yükseliyor. Varsayılan 180 saniye dolunca önce
Home'a dönüş başlıyor. İnişten önce yatay hata 0,5 m, yatay hız 0,4 m/s
altında en az bir simülasyon saniyesi bekliyorum. Bunlar simülasyon için
koyduğum eşikler; gerçek uçuş güvenliği iddiası değil.

Konum hazır değilse veya dönüş 90 saniyede tamamlanamazsa hatayı kaydedip
bulunduğu yerde iniş istiyorum. Bunu başarılı Home dönüşü saymıyorum.
Bu koruma otomatik engelden kaçınma değildir.

Otomatik kısa parkur için:

```bash
python3 scripts/fly_drone.py --demo
```

Kayıtlar her açılışta `artifacts/drone-flight` altında yeni bir dizine gidiyor.
Şehir dosyasını değiştirmiyorum. Şehirdeki sahne ve beş ışığı aynı değerlerle
uçuş kopyasına aktarıyorum. Terminalde birden fazla eski Gazebo oturumunu
açık tutmak bilgisayarın yükünü artırabilir; başlatıcı başka oturumları kapatmıyor.

## Sensör kaydı

Normal uçuş için Gazebo Python bağlayıcıları gerekmiyor. `--record-sensors`
kullanacaksam Gazebo kurulumuyla uyumlu Python'dan ayrı bir `.venv-gz`
ortamı hazırlıyorum. Bu makinede Homebrew Python 3.14 ile:

```bash
/opt/homebrew/bin/python3 -m venv --system-site-packages .venv-gz
.venv-gz/bin/python -c 'import gz.transport13; import gz.msgs10.image_pb2'
python3 scripts/fly_drone.py --record-sensors
```

Bu seçenekte şu an yalnız kalkış sırasında 8 saniyelik pencereyi kaydediyorum;
tüm parkuru kaydetmeyi sıradaki iş olarak ayırdım. Görüntü 640 × 480 / 10 Hz,
proje IMU'su 200 Hz ayarlı. Bilgisayarın yüküne bağlı duvar saati hızı ile
sensörün simülasyon saati hızını aynı kabul etmiyorum. Gerçek simülatör pozu
ayrı değerlendirme dosyasına gidiyor; kestirici girdisi olarak kullanılmıyor.

## Testler

```bash
PYTHONPATH=src python3 -m unittest discover -s tests
python3 scripts/check_home_return.py
```

İlk komut 55 birim test içeriyor. Iris kaynakları kurulmamışsa modele bağlı
bir test açıkça atlanıyor. İkinci komut kendi arayüzsüz simülasyonunda H'den
uzakta iniş, tekrar kalkış ve süre dolunca ilk Home'a dönüşü deniyor; kendi
oturumunu kapatıyor. Testte süreyi 45 s seçtim, normal uçuş varsayılanını
180 s bıraktım. Kontrol yaklaşık üç dakika sürebiliyor.

Daha önce derlediğim aynı kaynakları başka bir klasörde tutuyorsam:

```bash
python3 scripts/fly_drone.py --runtime-root "/yerel/derleme/klasoru"
python3 scripts/check_home_return.py --runtime-root "/yerel/derleme/klasoru"
```

Bu klasör `vendor/` ve `build/` içermeli; şehir her zaman bu depodaki
`gazebo/helsinki` dizininden okunuyor. Kaynak sürümlerinin aynı olmasını
kontrol ediyorum. Bu seçenek kişisel bir dosya yoluna bağımlı olmamak için var.

`don` için GUIDED konum hedefleri kullanıyorum. Bu test, ArduPilot RTL modunun
veya GNSS'siz dönüşün test edildiği anlamına gelmiyor. Şu an yalnız GPS açık
simülasyon uçuşunu doğruladım; gerçek donanımı bu çalışmaya dahil etmedim.
