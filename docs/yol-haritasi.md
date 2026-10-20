# Çalışma planım — 20 Ekim 2026

Amacım, GPS/GNSS bulunmadığında veya uçuş sırasında kesildiğinde kamera, IMU
ve önceden bilinen haritayla başlangıç noktasına dönebilen bir sistem geliştirmek.
Şu an GPS açıkken uçuşu ve ilk Home'a dönüşü doğruladım. Bu çalışmayı henüz
GPS'siz uçuş veya gerçek donanım başarısı olarak değerlendirmiyorum.

## Öncelikleri nasıl belirledim?

Haritayı daha fazla büyütmek yerine önce mevcut küçük alanda ölçülebilir
konumlama elde etmeye karar verdim. VIO'da ortalama hataya bakmakla
yetinmeyeceğim; ani sıçramaları ve takip kaybını da inceleyeceğim. GPS'i
kapatmadan önce alternatif kaynağın sağlıklı olduğunu göstereceğim.
İlk dönüşleri, engellerini önceden kontrol ettiğim açık bir koridorda
deneyeceğim. Home'a dönüş ile otomatik engelden kaçınmayı aynı şey saymıyorum.

## Görev sıram

| Adım | Ne yapacağım? | Sonraki adıma geçiş ölçütüm | Tahminim |
|---|---|---|---|
| 1 | Kamera, ham IMU, kalibrasyon ve zamanları tüm uçuş boyunca kaydedeceğim. Aynı kısa parkuru üç kez uçuracağım. | Kalkıştan inişe kadar okunabilir ve zamanları tutarlı kayıt; eksik örneklerin açık raporu. | 1–2 iş günü |
| 2 | Kamera + IMU hareket tahminini kararlı hâle getireceğim. Başlangıç, ölçek, kayma ve konum sıçramalarını inceleyeceğim. | Tekrarlı parkurlarda tutarlılık; takip kaybının bildirimi; kullanılabilir belirsizlik bilgisi. | 5–8 gün |
| 3 | Tahmini ArduPilot'a önce motorlar kapalı, sonra GPS açıkken pasif aktaracağım. | Eksenler, saat, hız ve kalite doğru; EKF kayıtlarında verinin kabulü görülebiliyor. | 3–5 gün |
| 4 | Küçük alanda kaynak geçişini, havada beklemeyi ve kısa GPS'siz hareketi test edeceğim. | GPS verisinin kullanılmadığı ve kontrolün alternatif konumla sürdüğü kayıtlardan doğrulanıyor. | 2–3 gün |
| 5 | Helsinki haritasından görüntüyle konum düzeltmesi ekleyeceğim. | Yalnız VIO'ya göre hata azalıyor; yanlış bölge eşleşmeleri reddediliyor. | 5–8 gün |
| 6 | GPS kesintisi, kaynak geçişi, ilk Home'a dönüş ve inişi birleştireceğim. | Farklı kesinti noktalarında Home korunuyor; bozuk konumla kontrol sürdürülmüyor. | 4–6 gün |
| 7 | Başlangıçtan GNSS'siz çalışma ve karşılaştırmalı deneyleri tamamlayacağım. | İlk konum/yön GPS'siz kurulabiliyor; sonuçları tekrar üretip raporlayabiliyorum. | 4–7 gün |

Günde yaklaşık 3–4 saat çalışarak, hata düzeltme payıyla 6–10 haftalık bir
simülasyon planı çıkardım. Bu kesin teslim tarihi değil. En belirsiz aşamam VIO'nun
kararlı çalışması. Aşama geçişlerini test sonucuna göre yapacağım; gerçek
donanım ve ürünleştirmeyi bu süreye dahil etmedim.

## Başarıyı nasıl ölçeceğim?

- Parkur boyunca konum ve yön hatasını, birikimli kaymayı ve Home'a iniş uzaklığını ölçeceğim.
- GPS kesintisinden kaynak geçişine kadar geçen süreyi kaydedeceğim.
- Görüntü işleme gecikmesini ve takip kayıplarını inceleyeceğim.
- Yanlış harita eşleşmesini, kamera/IMU kesilmesini, gecikmiş veriyi ve yeniden başlatmayı ayrı test edeceğim.
- GPS açık temel uçuşu, yalnız VIO'yu ve VIO + haritayı aynı koşullarda karşılaştıracağım.
- Deney koşullarını önceden belirleyip başarısız denemeleri de raporlayacağım.

Simülatörün gerçek konumunu yalnız değerlendirme tarafında kullanacağım.
GPS kesildikten sonra kestiriciye veya kontrol kararlarına GPS konumu/hızı
taşımayacağım. Bayat bir konumu yeni ölçülmüş gibi yayınlamayacağım.
Yükseklik ve yön için kullandığım barometre/pusula gibi yardımcı kaynakları
açıkça belirteceğim; GNSS'siz çalışmayı yalnız kamera + IMU iddiasıyla karıştırmayacağım.

Helsinki binaları ve zemin aynı düzlemde değil. Bu nedenle tek bir düzlemsel
görüntü dönüşümünü her yerde geçerli 3B drone konumu kabul etmeyeceğim;
haritanın 3B geometrisini ve kamera kalibrasyonunu dikkate alacağım.
Aynı model dokusundan üretilen görüntülerle aldığım sonucu gerçek dünya
genellemesi saymayacağım. Farklı bölge, ışık ve görüntü koşullarını ayrıca ayıracağım.

Konum kaynağı geçişi için [ArduPilot ExternalNav](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html)
ve [GPS/Non-GPS geçişleri](https://ardupilot.org/copter/docs/common-non-gps-to-gps.html)
belgelerini temel alacağım. Parametreleri kullandığım sabit kaynak sürümüyle
karşılaştırmadan değiştirmeyeceğim.
