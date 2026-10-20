# Güncelleme notlarım

## 20 Ekim 2026 — Drone uçuşu ve Home'a dönüş

- Helsinki ortamına Iris drone ekledim ve ArduPilot SITL ile uçurdum.
- Kalkış, kuzey/doğu hedefine gitme, bulunduğu yerde iniş ve Home'a dönüş komutlarını ekledim.
- H işaretli kalkış alanını hazırladım ve Gazebo'dan aldığım görüntüyü README'ye koydum.
- Home'u oturumun ilk başlangıç noktasında sabitledim. Başka yerde inip tekrar kalkınca başlangıç referansının değişmesi sorununu düzelttim.
- Uçuş süresi dolunca önce Home'a dönüş, ardından iniş yapılmasını sağladım.
- Home'a varınca konumun ve yatay hızın kararlı olmasını bekleyerek iniş başlattım.
- İki kalkışlı otomatik kontrol senaryosunu ve birim testleri ekledim.
- Kayıtlı ışık ayarlarımı korudum. Uçuş dünyasını ayrı bir çalışma kopyasında oluşturdum.
- Kurulum adımlarını, kullandığım kaynak sürümlerini ve sıradaki çalışma planımı yazdım.

Bu aşamayı GPS açıkken doğruladım. GPS'siz görsel konumlama ve geri dönüşü henüz tamamlamadım.
