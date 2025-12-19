import time
import pandas as pd  # Tablo için (yüklü değilse 'pip install pandas' yap)

# Kendi modüllerini import et
from multi_target_navigation.environment.grid import GridEnvironment
from multi_target_navigation.algorithms.bfs import BFSPlanner
from multi_target_navigation.algorithms.astar import AStarPlanner
from multi_target_navigation.algorithms.dijkstra import DijkstraPlanner
from multi_target_navigation.vrp.distance_matrix import compute_distance
from multi_target_navigation.vrp.greedy_vrp import greedy_vrp

def run_comparison():
    # --- 1. ORTAM KURULUMU (Zorlu bir harita) ---
    # Algoritmalar arasındaki farkı görmek için engelleri artırdık
    WIDTH, HEIGHT = 15, 15
    START = (1, 1)
    TARGETS = [(12, 12), (2, 10), (10, 2), (6, 6)]
    
    # "U" şeklinde engeller ve duvarlar
    OBSTACLES = [
        (3,3), (3,4), (3,5), (3,6), (3,7),
        (8,8), (8,9), (8,10), (7,8), (6,8),
        (10,5), (11,5), (12,5), (5,12), (5,13),
        (4, 8), (4, 9), (4, 10), (9, 3), (9, 4)
    ]

    env = GridEnvironment(WIDTH, HEIGHT, START, TARGETS, OBSTACLES)
    points = [env.start] + env.targets
    
    # Test edilecek algoritmalar
    planners = [
        ("BFS", BFSPlanner, "white"),
        ("Dijkstra", DijkstraPlanner, "cyan"),
        ("A*", AStarPlanner, "yellow")
    ]

    results = []

    print("\n--- ANALİZ BAŞLIYOR ---\n")

    for name, PlannerClass, color in planners:
        print(f"Algorithm running: {name}...")
        
        start_time = time.perf_counter()
        
        # 1. Mesafe Matrisi (VRP için tüm noktalar arası uzaklık)
        # Not: compute_distance fonksiyonu içinde 'method' stringi kullanılıyor.
        # İsimleri lower case yaparak gönderiyoruz. 'A*' -> 'astar'
        method_name = name.lower().replace("*", "star")
        dist_matrix = compute_distance(env.grid, points, method=method_name)
        
        # 2. Rota Hesaplama (Greedy VRP)
        order, total_path_cost = greedy_vrp(dist_matrix)
        
        # 3. Yolu Tekrar İnşa Etme ve Ziyaret Edilen Düğüm Sayısı (Nodes Visited) Analizi
        # Matris hesaplanırken düğümleri sayamıyoruz çünkü fonksiyon sadece matris dönüyor.
        # Bu yüzden rotayı çizerken gerçek performansı ölçeceğiz.
        
        full_path = []
        total_nodes_visited = 0
        current_pos = env.start
        
        # Sıralanmış hedefleri listeden al
        # order[0] her zaman start noktasıdır (index 0)
        ordered_targets_indices = [idx for idx in order if idx != 0]
        
        for t_idx in ordered_targets_indices:
            target_pos = points[t_idx]
            
            # Seçilen algoritma ile planlayıcıyı başlat
            planner = PlannerClass(env.grid, current_pos, target_pos)
            path_segment = planner.search()
            
            if path_segment:
                full_path.extend(path_segment[1:]) # Başlangıcı tekrar eklememek için
                total_nodes_visited += planner.nodes_visited # SAYAÇ BURADAN GELİYOR
                current_pos = target_pos
        
        end_time = time.perf_counter()
        elapsed_ms = (end_time - start_time) * 1000

        # Sonuçları kaydet
        results.append({
            "Algoritma": name,
            "Maliyet (Adım)": total_path_cost,
            "Gezilen Düğüm": total_nodes_visited,
            "Süre (ms)": f"{elapsed_ms:.2f}",
            "Rota Sırası": str(order)
        })

        # Sadece A* veya hepsi için görselleştirme yapabilirsin
        # Her biri için ayrı pencere açılacaktır.
        env.visualize(full_path, label=f"{name} + VRP", color=color)

    # --- TABLO ÇIKTISI ---
    df = pd.DataFrame(results)
    print("\n" + "="*60)
    print("KARŞILAŞTIRMALI ANALİZ SONUÇLARI")
    print("="*60)
    print(df.to_string(index=False))
    print("="*60 + "\n")

if __name__ == "__main__":
    run_comparison()