# Bitirme Projem — Gazebo Simülasyon Ortamı

Bitirme projem için Gazebo'da gerçek bir şehir alanından oluşan 3B bir ortam hazırladım. Bunun için Helsinki Belediyesi'nin paylaştığı Helsinki 3D Mesh 2017 verisini kullandım. Hazır modeldeki binaları, araziyi ve yüzey görüntülerini Gazebo'ya aktardım.

![Gazebo'da hazırladığım Helsinki ortamı](docs/images/gazebo-district.png)

## Neler yaptım?

- 1 × 1 km'lik bir alanı seçip harita parçalarını aynı koordinat sisteminde birleştirdim.
- Merkezde daha ayrıntılı model kullandım; küçük birleşim hatalarını ve yüzey normallerini düzenledim.
- Ortamın ışıklarını ayarlayıp dünya dosyasına kaydettim.
- Sabit bir test düzeneğine aşağı bakan kamera ve IMU ekledim, veri akışlarını kontrol ettim.
- Ortamı farklı açılardan açıp görüntüsünü kontrol ettim.

Bu repoda dünya dosyası, ortamı açmak için kullandığım Python kodu, yüzey işleme kodları ve ilgili testler bulunuyor. Büyük model ve doku dosyalarını [indirilebilir pakette](https://github.com/rabiasn/gnss-denied-drone-navigation/releases/tag/gazebo-helsinki-v6) paylaştım.

![Gazebo'da park ve ağaçlık alanlar](docs/images/gazebo-park.png)

## Ortamı açma

Ortamı MacBook'ta Gazebo Harmonic ile çalıştırdım. Açmak için Gazebo'nun (`gz` komutu) ve Python 3.10 veya üzerinin kurulu olması gerekiyor.

```bash
git clone https://github.com/rabiasn/gnss-denied-drone-navigation.git
cd gnss-denied-drone-navigation
python3 scripts/download_gazebo_assets.py
python3 scripts/open_gazebo_world.py --view district
```

İndirme komutu modelleri ve dokuları indiriyor, açılış komutu ortamı açıyor. Harita büyük olduğu için ilk yükleme biraz sürebiliyor. Kapatmak için terminalde `Ctrl+C` kullanıyorum. Genel görünüm için `--view district` yerine `--view overview` yazılabilir.

Işık ayarlarım `gazebo/helsinki/world.sdf` dosyasında kayıtlı. Arayüzde yaptığım yeni ayarları saklamak için **Save world as** seçeneğini kullanıyorum.

## Veri kaynağı ve lisans

Kullandığım model ve dokular Helsinki Belediyesi'ne ait ve **CC BY 4.0** lisanslı. Model üzerinde koordinat dönüşümü, yüzey ve parça birleşimi düzenlemeleri yaptım. Fotoğraflar, Gazebo'da açtığım ortamdan aldığım ekran görüntüleri.

- [Helsinki Belediyesi — Helsinki 3D](https://www.hel.fi/en/decision-making/information-on-helsinki/maps-and-geospatial-data/helsinki-3d)
- [CC BY 4.0 lisansı](https://creativecommons.org/licenses/by/4.0/)
- [Kaynak ve değişiklik bilgileri](gazebo/helsinki/ATTRIBUTION.txt)
