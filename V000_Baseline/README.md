# Swarm Drone Coverage Simulator — Clean Legacy A* Baseline

> **Status:** baseline yang telah dibersihkan, dipertahankan untuk eksperimen lanjutan. GUI mengikuti versi visualisasi V010–V013. **Bukan** implementasi final *Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy*.

## 1. Ikhtisar 5W1H

| Aspek | Penjelasan |
|---|---|
| **What — Apa?** | Simulator eksplorasi cakupan oleh beberapa drone pada lingkungan grid 2D dengan *frontier detection*, *exclusive frontier ownership*, dan pencarian rute A* menuju frontier milik tiap drone. |
| **Why — Mengapa?** | Menyediakan baseline yang mudah dibaca untuk menguji sensing, navigasi, capaian observasi, dan redundansi sebelum membuat algoritma penelitian yang lebih kompleks. |
| **Who — Siapa?** | Peneliti/mahasiswa yang mempelajari robotika swarm, eksplorasi grid, perencanaan gerak dan evaluasi multi-agen. |
| **Where — Di mana?** | Aplikasi desktop JavaFX; simulasi berlangsung pada grid dua dimensi yang digambar atau dibuka dari file `.sc`. |
| **When — Kapan digunakan?** | Saat merancang lingkungan, memulai Run, membandingkan lintasan hasil eksplorasi, dan sebagai benchmark untuk versi berikutnya. |
| **How — Bagaimana?** | Sensor radial memperbarui peta observasi **bersama**; frontier known-free yang berbatasan dengan unknown diberikan ke owner; A* mencari frontier milik agen yang terdekat; gerakan berlangsung sinkron; metrik dihitung ketika simulasi berhenti. |

## 2. Status dan batasan ilmiah — wajib dibaca

Versi ini adalah **legacy A* benchmark**, bukan algoritma desentralisasi dan bukan sistem repulsive pheromone aktif.

- Satu occupancy map diketahui bersama oleh semua drone melalui `DecentralizedExploration.map`.
- Satu matriks `owner` mengalokasikan frontier secara eksklusif; alokasi ini dilakukan oleh engine bersama, bukan keputusan otonom tiap drone.
- A* menggunakan `truth` (ground-truth obstacle layout) untuk pemeriksaan *traversability*. Ini **oracle-assisted navigation**. Peta hasil sensor hanya dipakai untuk menemukan frontier.
- Tidak ada pheromone deposit, evaporation, pheromone-based target utility, local pheromone memory, atau inter-drone message protocol.
- `Drone` pada baseline bersih **hanya representasi/visualisasi**, bukan controller mandiri. Ini disengaja agar metode tidak aktif tidak terlihat seolah-olah tersedia.
- Tidak ada klaim bahwa *overlap* atau *revisit* akan selalu nol.
- Hasil selesai mengobservasi area (`observed area = 100%`) berbeda dari kunjungan fisik ke seluruh sel (`visited coverage`).

Untuk artikel **“Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy”**, kode ini dapat dijadikan **benchmark pembanding**, bukan implementasi metode *proposed*.

## 3. Struktur proyek

```text
SwarmCoverage_CleanBaseline/
├── README.md
├── CHANGELOG_CLEANUP.md
├── src/main/java/swarmdrone/decentralizedswarmcoverage_ijrcs/
│   ├── Launcher.java
│   ├── MainView.java
│   ├── CanvasView.java
│   ├── DecentralizedExploration.java
│   ├── Drone.java
│   ├── Point.java
│   ├── State.java
│   ├── CSS.java
│   └── SVGIcon.java
└── tests/
    ├── regression_core.sh
    ├── RegressionHarness.java
    └── stubs/ (JavaFX minimal untuk tes algoritma saja)
```

> File resource `icon.png`, konfigurasi proyek IntelliJ/Maven/Gradle, dan dependensi JavaFX **tidak termasuk** dalam sembilan source upload. Gunakan resource `icon.png` dan konfigurasi JavaFX dari proyek Anda yang telah berjalan. Kelas `MainView` tetap memuat icon dari resource package yang sama.

## 4. Keterkaitan antarclass

```text
Launcher.main()
  -> JavaFX Application.launch(MainView.class)
      -> MainView.start(): siapkan window, tombol, AnimationTimer
          -> CanvasView: editor peta, drone, zoom/pan, file .sc, replay
              -> CanvasView.run()
                  -> new DecentralizedExploration(environment, drones)
                      -> markReachable()         [ground-truth evaluator]
                      -> simulate()
                         -> sense() / discover() [shared observed map]
                         -> detectOwnedFrontiers() / isFrontier()
                         -> astarNearest() / heuristic() / legal()
                         -> simulasi gerakan serempak + collision handling
                         -> visit(), simpan Drone.path, hitung metrik
              -> CanvasView.update() + render()
                   -> Drone.draw() / drawSensoreVisibility()
                   -> warna lintasan lewat Drone.getPathColor()
```

### Tanggung jawab class

| Class | Method/data penting | Tanggung jawab |
|---|---|---|
| `Launcher` | `main()` | Menjalankan JavaFX. |
| `MainView` | `start()`, `handleButtonRun()`, `handleButtonOpen()`, `handleButtonSave()`, `handleButtonPlayPause()`, `handleButtonReplay()` | GUI, toolbar, error dialog, pengikatan event. |
| `CanvasView` | `run()`, `render()`, `update()`, `readFromFile()`, `writeToFile()`, `handleMouse*()` | Editor lingkungan, replay sinkron, gambar grid, obstacle, sensor, trajectory. |
| `DecentralizedExploration` | `simulate()`, `sense()`, `discover()`, `detectOwnedFrontiers()`, `isFrontier()`, `astarNearest()`, `visit()` | Seluruh algoritma benchmark dan evaluasi. Nama class dipertahankan untuk kompatibilitas proyek. |
| `Drone` | `draw()`, `drawSensoreVisibility()`, `path`, `frameIndex`, `setPathColorByIndex()` | Posisi awal, 30 warna lintasan, data lintasan, dan visualisasi. |
| `Point` | `getRow()`, `getCol()`, `equals()`, `hashCode()` | Koordinat grid integer yang immutable. |
| `State` | `DEFAULT`, `OBSTACLE`, `LINE`, `REMOVE_LINE`, `ADD_DRONE`, `REMOVE_DRONE` | Mode interaksi editor. |
| `CSS` | `style` | Stylesheet JavaFX seperti pada sumber asli. |
| `SVGIcon` | Data SVG tombol | Ikon toolbar, mengikuti visualisasi asli. |

## 5. Cara kerja algoritma

### 5.1. Representasi occupancy

Peta hasil sensing bersama `map[r][c]`: `0=UNKNOWN`, `1=KNOWN_FREE`, `-1=KNOWN_OBSTACLE`. `truth[r][c]` adalah peta lingkungan asli yang dipakai simulator, termasuk pada A* benchmark saat ini.

### 5.2. Sensing radial

Setiap langkah, setiap drone memancarkan 360 ray. Ray berakhir pada obstacle atau batas radius sensor (`sensorFoV=4` sel di `CanvasView`, ditambah 0,5 sel pada sampling). `discover()` mengubah unknown menjadi free/occupied; `firstObservedAt` menyimpan timestep pertama sel reachable free terlihat. **Render sensor di `Drone.drawSensoreVisibility()` merupakan komponen visual yang terpisah dari raycasting algoritma**, meskipun keduanya memakai konsep radial yang sama.

### 5.3. Frontier detection & ownership

Frontier adalah sel **KNOWN_FREE** yang setidaknya memiliki satu tetangga 8-connected **UNKNOWN**. `detectOwnedFrontiers()` memberikan ownership ke drone pertama dalam urutan pemrosesan yang mendeteksi sel tersebut dalam radius. Ownership dilepas ketika sel tidak lagi frontier. Ini merupakan **global exclusive ownership**, bukan lease/claim desentralisasi.

### 5.4. A* ke nearest owned frontier

`astarNearest(id)` memilih rute terpendek menuju salah satu frontier yang dimiliki drone. Dalam grid 8-connected:

- Gerakan horizontal/vertikal: biaya `1`.
- Gerakan diagonal: biaya `sqrt(2)`.
- Heuristic: **octile distance**, minimum terhadap seluruh candidate goals.
- Diagonal *corner cutting* dilarang jika salah satu sel samping terhalang.
- **Catatan:** pemeriksaan sel bebas tetap memakai ground truth, bukan occupancy yang baru diamati.

### 5.5. Simultaneous movement & collision handling

Drone mengusulkan satu sel berikutnya setiap timestep. Engine memeriksa destination conflict (prioritas indeks drone kecil), head-on swap, memasuki sel drone diam, dan persilangan diagonal. Posisi yang ditolak dicatat sebagai langkah menunggu agar semua `Drone.path` memiliki satu entri per timestep.

### 5.6. Termination

Simulasi berakhir bila semua reachable free cells telah **diobservasi** (bukan harus dikunjungi), tidak ada perpindahan selama 30 timestep, atau mencapai `swarm.legacy.maxSteps` (default 2500). Mask reachable dihitung simulator memakai ground truth untuk evaluasi.

## 6. Metrik

- `getObservedRatio()` = `observedReachable / reachableFree * 100`.
- `getCoverage()` = `unique physically visited free cells / reachableFree * 100`.
- `getRevisit()` = `(totalVisits - uniqueVisits) / totalVisits * 100`.
- `getOverlap()` = `cells visited by >=2 distinct drones / uniqueVisits * 100`.
- `getSteps()` = jumlah timestep gerakan hingga berhenti; **bukan** waktu runtime program atau detik animasi.
- `getTerminationReason()` = alasan berhenti.

Kunjungan awal setiap drone dihitung satu kali. Waktu diam tidak ditambah sebagai kunjungan baru. Metrik overlap merupakan **physical visitation overlap**, bukan overlapped sensor footprints.

## 7. Visualisasi dan fitur aplikasi

Tampilan aktif disimpan seperti pada sumber upload: grid dan obstacle turquoise, lingkungan explored/unexplored, drone dengan radius sensor oranye, nama/koordinat, **30 pilihan warna path**, zoom, pan, tambah/hapus drone, obstacle/line editor, serta tombol **New**, **Open**, **Save**, **Run**, **Play/Pause**, dan **Replay**.

Saat tombol **Run** diklik, perhitungan seluruh eksplorasi berlangsung terlebih dahulu di konstruktor `DecentralizedExploration`; selanjutnya `AnimationTimer` menampilkan playback dari `Drone.path`. Penghitung playback berada di `CanvasView`, sehingga seluruh drone tetap sinkron.

## 8. Cara menjalankan

1. Gunakan **JDK 21** dan project **JavaFX** yang telah Anda gunakan sebelumnya (alternatif minimal JDK 17 karena kode menggunakan Java record).
2. Tempatkan sembilan file `.java` sesuai package `swarmdrone.decentralizedswarmcoverage_ijrcs`.
3. Pertahankan `pom.xml`/Gradle serta `icon.png` dari proyek Anda yang berfungsi. Resource `icon.png` harus berada pada package/resource path yang dapat ditemukan `MainView.class.getResourceAsStream("icon.png")`.
4. Jalankan `Launcher.main()` di IntelliJ atau jalankan konfigurasi JavaFX proyek Anda.
5. Tambahkan drone pada sel bebas dan gambar/buka peta (`.sc`); klik **Run**.
6. Gunakan **Play/Pause** atau **Replay** untuk memeriksa riwayat trajectory dan observed area.

Contoh VM Options untuk batas simulasi (opsional):

```text
-Dswarm.legacy.maxSteps=2500
```

### Berkas `.sc`

Format text yang dibaca `CanvasView`: header jumlah baris/kolom; baris grid (`0` ruang bebas, `-1` obstacle); opsional daftar posisi awal drone (baris, kolom). Perhatikan format koordinat yang ditampilkan sebagai `(row+1, col+1)` di GUI sementara format file memakai indeks berbasis nol.

## 9. Regression test dan tingkat verifikasi

Uji algoritma tanpa GUI (Linux/macOS dengan `bash` dan `javac`):

```bash
bash tests/regression_core.sh
```

Script menggunakan **minimal stub JavaFX** hanya untuk kompilasi kelas inti (`Drone`, `DecentralizedExploration`, `Point`), menjalankan skenario sintetis dua drone, kemudian memeriksa observed ratio, timestep, metrik, dan hash lintasan yang telah ditetapkan. Stub **tidak memvalidasi rendering JavaFX secara langsung**.

Saat cleaning, engine sebelum dan sesudah menghasilkan hasil dan lintasan sama persis pada satu skenario deterministik dua drone; ini **bukan** bukti kesetaraan pada semua peta. Jalankan GUI langsung di IntelliJ untuk menguji interaksi mouse, seluruh tombol, resource icon, dan Open/Save.

## 10. Rekomendasi pengembangan sesudah baseline

Pengembangan artikel utama nanti sebaiknya dilakukan sebagai cabang/versi baru agar baseline benchmark tidak berubah. Untuk benar-benar memenuhi judul penelitian, dibutuhkan:

1. **Peta lokal privat** dan frontier selection secara otonom pada masing-masing drone (tidak menggunakan `owner` global untuk pembagian target).
2. **A* atas known-free local map**, bukan `truth` yang belum diketahui drone.
3. **Digital pheromone** yang terdeposit pada sel yang dikunjungi, menguap per timestep, diamati secara lokal, dan memberi penalti eksplisit pada skor frontier.
4. **Ablation study** yang memisahkan efek nearest frontier, information gain, map exchange/claiming, dan repulsive pheromone.
5. Beberapa geometri, posisi awal, jumlah drone dan seed, disertai pelaporan observed completion, visited coverage, revisit, overlap, distance, dan waktu.

Sampai komponen tersebut benar-benar dipakai oleh engine, **jangan menyebut baseline ini communication-free decentralized pheromone exploration**.
