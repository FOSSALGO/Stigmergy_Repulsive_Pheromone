# Swarm Drone Coverage Simulator — Legacy Frontier Exploration with A*

Simulator eksplorasi *swarm drone* berbasis grid menggunakan **Java dan JavaFX**. Versi ini menggabungkan antarmuka visual dari seri V010–V012 dengan pendekatan eksplorasi dari `SwarmDroneSimulator.java`, kemudian mengganti pencarian jalur BFS dengan **A* berbobot jarak geometris**.

> **Status: experimental / legacy-algorithm benchmark.** Implementasi ini menggunakan peta okupansi bersama dan informasi obstacle sebenarnya untuk perencanaan jalur (*oracle navigation*). Meskipun class bernama `DecentralizedExploration`, versi ini **bukan implementasi eksplorasi yang sepenuhnya terdesentralisasi**. **Repulsive digital pheromone belum digunakan dalam pemilihan frontier atau A*.** Gunakan sebagai pembanding, bukan sebagai bukti implementasi lengkap metode pada artikel *“Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy.”*

## Fitur utama

- **Frontier-based coverage exploration:** kandidat frontier adalah sel yang sudah diketahui bebas dan memiliki setidaknya satu tetangga 8-connected yang belum diketahui.
- **Exclusive frontier ownership:** frontier yang ditemukan dialokasikan kepada satu drone melalui daftar kepemilikan bersama; frontier diperbarui ketika status eksplorasi berubah.
- **A* to nearest owned frontier:** memilih frontier milik drone yang dapat dicapai dengan biaya lintasan terendah.
- **8-connected grid:** perpindahan horizontal/vertikal dan diagonal, dengan pembatasan *diagonal corner-cutting* melalui obstacle.
- **360° radial sensing:** obstacle menghalangi observasi sel di belakangnya.
- **Synchronous simulation:** setiap drone bergerak paling banyak satu sel per timestep, disertai penanganan konflik gerak.
- **Replay visualization:** lintasan masing-masing drone memiliki warna berbeda, area belum diamati ditampilkan lebih gelap, dan histori simulasi dapat diputar ulang.
- **Environment editor:** membuat/menghapus drone, menggambar/menghapus obstacle, zoom, pan, serta membuka dan menyimpan file `.sc`.

## Algoritma A* dan jarak geometris

Biaya langkah pada grid berbentuk persegi adalah:

| Perpindahan | Biaya dalam satuan sisi sel |
| --- | ---: |
| Kanan, kiri, atas, bawah | `1` |
| Empat arah diagonal | `√2 ≈ 1.4142` |

Jika panjang sisi sel adalah `s` meter, biaya sebenarnya adalah `s` meter untuk langkah lurus dan `s√2` meter untuk diagonal. Ukuran `cellSize = 10` di source mengatur **ukuran tampilan piksel**, bukan secara otomatis mendefinisikan ukuran sel fisik dalam meter.

Untuk titik `(r,c)` dan tujuan `(rg,cg)`, A* menggunakan heuristik *octile distance*:

```text
dr = abs(r - rg)
dc = abs(c - cg)
h  = max(dr, dc) + (sqrt(2) - 1) * min(dr, dc)
```

A* menggunakan `g` (akumulasi biaya langkah) dan `h` (estimasi sisa jarak), bukan jumlah langkah BFS. Perencanaan rute pada **versi benchmark ini** menggunakan obstacle dari peta lingkungan sebenarnya untuk memvalidasi lintasan, sebagaimana asumsi pada algoritma lama.

## Cara kerja simulasi

1. Pengguna menggambar atau membuka environment dan menempatkan drone.
2. Simulator menentukan sel bebas yang dapat dijangkau dari posisi awal untuk kebutuhan evaluasi.
3. Setiap timestep, semua drone melakukan observasi radial dan memperbarui **peta okupansi bersama**.
4. Sistem mendeteksi frontier dan menyimpan kepemilikan frontier masing-masing drone.
5. Setiap drone memilih jalur A* ke frontier terdekat yang dimilikinya.
6. Drone mengusulkan perpindahan satu sel; konflik perpindahan diselesaikan sebelum posisi diterapkan.
7. Pergerakan, kunjungan sel, dan waktu observasi pertama dicatat untuk metrik dan replay.
8. Simulasi berhenti setelah semua sel bebas yang dapat dijangkau terobservasi, terjadi *stall*, atau batas timestep tercapai.

**Penting:** tidak semua sel harus dikunjungi secara fisik agar area dapat dinyatakan terobservasi. *Observed-area completion* dan *visited coverage* merupakan dua ukuran berbeda.

## Struktur source code

Semua file Java menggunakan package:

```java
package swarmdrone.decentralizedswarmcoverage_ijrcs;
```

```text
src/main/java/swarmdrone/decentralizedswarmcoverage_ijrcs/
├── Launcher.java                  # Entry point JavaFX
├── MainView.java                  # Jendela utama, toolbar, dan kontrol
├── CanvasView.java                # Editor grid, visualisasi, playback, Open/Save
├── DecentralizedExploration.java  # Mesin benchmark: shared map, frontier, A*, metrik
├── Drone.java                     # Model drone, rendering, sensor dan palet lintasan
├── Point.java                     # Representasi koordinat grid
├── State.java                     # Mode interaksi editor
├── CSS.java                       # Gaya antarmuka JavaFX
└── SVGIcon.java                   # Ikon toolbar
```

> Nama `DecentralizedExploration.java` dipertahankan untuk kompatibilitas dengan antarmuka aplikasi. Isi kelas pada versi ini **menggunakan pengaturan frontier bersama dan oracle navigation**. `Drone.java` masih menyertakan sejumlah method dari versi penelitian sebelumnya, tetapi perhitungan eksplorasi benchmark dijalankan oleh `DecentralizedExploration.java`.

## Persyaratan dan menjalankan aplikasi

- **JDK 21** direkomendasikan.
- **JavaFX SDK** dengan modul `javafx.controls` (serta modul lain jika diperlukan oleh konfigurasi proyek).
- IDE seperti **IntelliJ IDEA** atau lingkungan Java yang dapat menjalankan JavaFX.
- Source yang dibagikan di sini berupa file `.java`; **file build `pom.xml`/`build.gradle` dan resource ikon belum disertakan**, sehingga gunakan konfigurasi JavaFX dan resource dari project yang sudah berjalan, atau tambahkan sendiri.

### Melalui IntelliJ IDEA

1. Buka project JavaFX yang sudah dikonfigurasi.
2. Salin sembilan class ke package `swarmdrone.decentralizedswarmcoverage_ijrcs` tanpa mengubah nama class.
3. Pastikan dependensi JavaFX tersedia dan resource `icon.png` berada pada classpath yang dapat diakses `MainView` (atau sesuaikan pemuatan ikon bila resource belum tersedia).
4. Jalankan class `Launcher` sebagai *main class*.
5. Buat environment atau buka file `.sc`, tambahkan drone, kemudian klik **Run**.
6. Gunakan **Play/Pause** atau **Replay** untuk meninjau lintasan eksplorasi.

Apabila JavaFX menggunakan SDK manual, VM options biasanya perlu memuat `--module-path` yang menunjuk folder `lib` SDK setempat dan `--add-modules javafx.controls`. Lokasi path harus disesuaikan dengan komputer Anda.

### Parameter simulasi

Batas timestep default berasal dari:

```java
private static final int MAX_STEPS =
        Integer.getInteger("swarm.legacy.maxSteps", 2500);
```

Untuk menggantinya, tambahkan contoh berikut pada VM Options:

```text
-Dswarm.legacy.maxSteps=5000
```

Radius sensor default di `CanvasView.java` adalah **4 sel**. Interval playback sekitar **95 ms per timestep**. Parameter ini merupakan konfigurasi implementasi, bukan hasil optimasi penelitian.

## Tombol antarmuka

| Kontrol | Fungsi |
| --- | --- |
| **New** | Mengosongkan dan mengatur ulang environment |
| **Open** | Membuka file environment `.sc` |
| **Save** | Menyimpan environment dan posisi awal drone ke `.sc` |
| **Transform** | Memilih/memindahkan drone dan melakukan navigasi canvas |
| **Add Drone** | Menambahkan drone dengan warna lintasan yang dialokasikan saat dibuat |
| **Remove Drone** | Menghapus drone pada posisi awalnya |
| **Wall / Draw Line / Remove Line** | Mengedit obstacle |
| **Run** | Menghitung eksplorasi lalu memulai playback |
| **Play/Pause** | Menghentikan atau melanjutkan animasi |
| **Replay** | Memutar ulang simulasi dari timestep awal |

Warna lintasan diambil dari palet **30 warna** pada `Drone.java`. Zoom menggunakan scroll mouse, sementara panning didukung melalui drag tombol tengah mouse.

## Format file `.sc`

Editor menyimpan dimensi environment, nilai sel, dan posisi awal drone. Contoh sederhana:

```text
ENVIRONMENT----------------------------------
NUM ROWS:5
NUM COLS:5
-1:-1:-1:-1:-1:
-1:0:0:0:-1:
-1:0:0:0:-1:
-1:0:0:0:-1:
-1:-1:-1:-1:-1:
DRONES---------------------------------------
NUM DRONES:2
1:1
3:3
```

Keterangan:

- `-1` = obstacle/wall.
- `0` = sel bebas dalam file environment.
- Koordinat drone disimpan sebagai `row:col` dengan indeks **dimulai dari nol**.
- Nilai `1` untuk *known free* dipakai pada peta eksplorasi internal, bukan sebagai nilai environment yang disimpan oleh editor.

## Metrik evaluasi

Mesin simulasi menyediakan metrik berikut:

| Metrik | Definisi |
| --- | --- |
| **Observed ratio (%)** | Jumlah sel bebas terjangkau yang telah diamati sensor / total sel bebas terjangkau × 100 |
| **Visited coverage (%)** | Jumlah sel bebas unik yang dikunjungi drone / total sel bebas terjangkau × 100 |
| **Revisit rate (%)** | `(totalVisits - uniqueVisits) / totalVisits × 100` |
| **Overlap rate (%)** | Sel unik yang dikunjungi oleh lebih dari satu drone / seluruh sel unik yang dikunjungi × 100 |
| **Completion time** | Jumlah timestep sampai kondisi terminasi |

Metrik tercetak melalui `System.out.printf` di konsol ketika simulasi selesai. Posisi awal juga dicatat sebagai kunjungan pertama. Timestep ketika drone menunggu tetap direkam pada histori replay, tetapi tidak menambah penghitung kunjungan fisik.

Kemungkinan nilai alasan terminasi: `ALL_REACHABLE_FREE_CELLS_OBSERVED`, `STALL_30_STEPS`, `MAX_STEPS`, dan `NO_DRONES`.

## Batasan penting untuk reproduksibilitas

1. **Bukan pure decentralized exploration.** Frontier ownership dan occupancy map dikelola bersama dalam mesin eksplorasi.
2. **Oracle navigation.** A* menggunakan informasi sel bebas/obstacle dari environment asli untuk validitas lintasan. Ini bukan A* yang hanya bergantung pada *private local map*.
3. **Tidak ada repulsive pheromone dalam keputusan.** Array `pheromone` ada untuk kompatibilitas antarmuka, tetapi tidak diperbarui menjadi jejak eksplorasi dan tidak memengaruhi target.
4. **Tidak ada jaminan zero redundancy.** Exclusive frontier ownership menghindari duplikasi kepemilikan target tertentu, tetapi tidak menjamin semua lintasan fisik tidak berpotongan atau tidak mengunjungi sel yang sama.
5. **Bukan pengukuran dunia nyata.** Sensing, gerak, jarak, dan konflik disimulasikan pada grid; performa robot fisik memerlukan pemodelan tambahan.
6. **UI dan core terintegrasi secara sinkron.** Simulasi penuh dihitung ketika **Run** dipanggil, sedangkan playback menampilkan histori tersebut sesudahnya.

Karena itu, hasil benchmark ini harus dipisahkan dari eksperimen algoritma utama penelitian **“Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy.”** Untuk menguji kontribusi penelitian tersebut, diperlukan versi yang benar-benar menggunakan keputusan lokal dan penalti pheromone dalam frontier utility.

## Arah pengembangan

Benchmark ini dapat digunakan untuk membandingkan jalur **A* nearest-frontier** dengan implementasi penelitian yang menggunakan *local occupancy map*, *repulsive pheromone* dan koordinasi swarm. Perbandingan sebaiknya menggunakan peta, jumlah drone, posisi awal, radius sensor, serta definisi metrik yang identik, sembari secara eksplisit melaporkan perbedaan akses informasi global.

## Kredit

Dikembangkan untuk eksperimen simulasi eksplorasi swarm drone dalam konteks **Robotics & Mechatronics Laboratory, Mechanical Engineering, Hasanuddin University**.

Tidak ada file lisensi yang disertakan bersama sembilan source code yang diperiksa. Tambahkan `LICENSE` secara terpisah jika repositori akan didistribusikan dengan lisensi tertentu.
