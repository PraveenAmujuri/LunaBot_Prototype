# ⚡ System Performance Benchmarks

Performance metrics were captured during full autonomous operation  
(SLAM + YOLO + Navigation active) running on Lenovo LOQ 15IAx9.

## 1. Resource Utilization (`rqt_top`)
The system demonstrates high efficiency with significant headroom.

![Process Monitor](images/rqt_top.png)

- **Total CPU Load:** ~10–15%  
- **SLAM (rtabmap):** ~2–3%  
- **WebSocket Layer:** < 2%  
- **Navigation Nodes:** < 1%  

## 2. Latency & Throughput
- **TF Latency:** 0.00s (static transforms)  
- **LiDAR Rate:** 10 Hz  
- **Depth Camera Rate:** 30 FPS (32FC1)  
- **RGB Camera Rate:** Adjustable (default 15–30 FPS)  

## 3. AI Inference Speed
- **YOLOv8 Inference:** 12–15 ms per frame  
- **Full Perception Pipeline:** < 50 ms end-to-end  

## 4. Development Velocity
- **Time to MVP:** 20 days  
- **Lines of Code:** ~15,000  
- **Custom Scripts:** ~2,500 Python + C#  

