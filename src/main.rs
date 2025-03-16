use collision_test::{
    static_collision::StaticCollision, 
    dynamic_collision::DynamicCollision,
    *
};


const LOOP_COUNT: usize = 10;


fn main() {
    // 1. AABB vs AABB
    {
        let main_collider = BoundingBox::new(
            glam::Vec3::new(0.0, 0.0, 0.0),
            glam::Vec3::new(1.0, 1.0, 1.0),
        );

        let mut colliders = Vec::new();
        for _ in 0..1_000_000 {
            let x = rand::random::<f32>() * 100.0 - 50.0;
            let y = rand::random::<f32>() * 100.0 - 50.0;
            let z = rand::random::<f32>() * 100.0 - 50.0;
            let ex = rand::random::<f32>() * 5.0 + 1.0;
            let ey = rand::random::<f32>() * 5.0 + 1.0;
            let ez = rand::random::<f32>() * 5.0 + 1.0;
            let collider = BoundingBox::new(
                glam::Vec3::new(x, y, z),
                glam::Vec3::new(ex, ey, ez),
            );
            colliders.push(collider);
        }

        {
            let mut avg_elapsed = std::time::Duration::new(0, 0);
            let mut avg_fps = 0.0;
            for _ in 0..LOOP_COUNT {
                let start = std::time::Instant::now();

                for collider in colliders.iter() {
                    let _ = main_collider.check_static_collision(collider);
                }

                let elapsed = start.elapsed();
                let fps = 1.0 / elapsed.as_secs_f32();
                avg_elapsed += elapsed;
                avg_fps += fps;
                println!("AABB vs AABB (SAT): {:<8.2?} (fps: {})", elapsed, fps);
            }
            avg_elapsed /= LOOP_COUNT as u32;
            avg_fps /= LOOP_COUNT as f32;
            println!("AABB vs AABB (SAT) avg: {:<8.2?} (fps: {})", avg_elapsed, avg_fps);
        }

        {
            let mut avg_elapsed = std::time::Duration::new(0, 0);
            let mut avg_fps = 0.0;
            for _ in 0..LOOP_COUNT {
                let start = std::time::Instant::now();

                for collider in colliders.iter() {
                    let _ = main_collider.gjk(collider);
                }

                let elapsed = start.elapsed();
                let fps = 1.0 / elapsed.as_secs_f32();
                avg_elapsed += elapsed;
                avg_fps += fps;
                println!("AABB vs AABB (GJK): {:<8.2?} (fps: {})", elapsed, fps);
            }
            avg_elapsed /= LOOP_COUNT as u32;
            avg_fps /= LOOP_COUNT as f32;
            println!("AABB vs AABB (GJK) avg: {:<8.2?} (fps: {})", avg_elapsed, avg_fps);
        }

        {
            let mut avg_elapsed = std::time::Duration::new(0, 0);
            let mut avg_fps = 0.0;
            for _ in 0..LOOP_COUNT {
                let start = std::time::Instant::now();

                for collider in colliders.iter() {
                    let _ = main_collider.aabb_collision_details(collider);
                }

                let elapsed = start.elapsed();
                let fps = 1.0 / elapsed.as_secs_f32();
                avg_elapsed += elapsed;
                avg_fps += fps;
                println!("AABB vs AABB (SAT-with normal): {:<8.2?} (fps: {})", elapsed, fps);
            }
            avg_elapsed /= LOOP_COUNT as u32;
            avg_fps /= LOOP_COUNT as f32;
            println!("AABB vs AABB (SAT-with normal) avg: {:<8.2?} (fps: {})", avg_elapsed, avg_fps);
        }

        {
            let mut avg_elapsed = std::time::Duration::new(0, 0);
            let mut avg_fps = 0.0;
            for _ in 0..LOOP_COUNT {
                let start = std::time::Instant::now();

                for collider in colliders.iter() {
                    let _ = main_collider.gjk_epa(collider);
                }

                let elapsed = start.elapsed();
                let fps = 1.0 / elapsed.as_secs_f32();
                avg_elapsed += elapsed;
                avg_fps += fps;
                println!("AABB vs AABB (GJK-EPA): {:<8.2?} (fps: {})", elapsed, fps);
            }
            avg_elapsed /= LOOP_COUNT as u32;
            avg_fps /= LOOP_COUNT as f32;
            println!("AABB vs AABB (GJK-EPA) avg: {:<8.2?} (fps: {})", avg_elapsed, avg_fps);
        }
    }

    println!();

    // 2. OBB vs OBB
    {
        let main_collider = BoundingBox::new_rotated(
            glam::Vec3::new(0.0, 0.0, 0.0),
            glam::Vec3::new(1.0, 1.0, 1.0),
            glam::Mat3::from_rotation_y(45.0_f32.to_radians()),
        );

        let mut colliders = Vec::new();
        for _ in 0..1_000_000 {
            let x = rand::random::<f32>() * 100.0 - 50.0;
            let y = rand::random::<f32>() * 100.0 - 50.0;
            let z = rand::random::<f32>() * 100.0 - 50.0;
            let ex = rand::random::<f32>() * 5.0 + 1.0;
            let ey = rand::random::<f32>() * 5.0 + 1.0;
            let ez = rand::random::<f32>() * 5.0 + 1.0;
            let rx = rand::random::<f32>() * 360.0;
            let ry = rand::random::<f32>() * 360.0;
            let rz = rand::random::<f32>() * 360.0;
            let collider = BoundingBox::new_rotated(
                glam::Vec3::new(x, y, z),
                glam::Vec3::new(ex, ey, ez),
                glam::Mat3::from_euler(glam::EulerRot::YXZ, rx.to_radians(), ry.to_radians(), rz.to_radians()),
            );
            colliders.push(collider);
        }

        {
            let mut avg_elapsed = std::time::Duration::new(0, 0);
            let mut avg_fps = 0.0;
            for _ in 0..LOOP_COUNT {
                let start = std::time::Instant::now();

                for collider in colliders.iter() {
                    let _ = main_collider.check_static_collision(collider);
                }

                let elapsed = start.elapsed();
                let fps = 1.0 / elapsed.as_secs_f32();
                avg_elapsed += elapsed;
                avg_fps += fps;
                println!("OBB vs OBB (SAT): {:<8.2?} (fps: {})", elapsed, fps);
            }
            avg_elapsed /= LOOP_COUNT as u32;
            avg_fps /= LOOP_COUNT as f32;
            println!("OBB vs OBB (SAT) avg: {:<8.2?} (fps: {})", avg_elapsed, avg_fps);
        }

        {
            let mut avg_elapsed = std::time::Duration::new(0, 0);
            let mut avg_fps = 0.0;
            for _ in 0..LOOP_COUNT {
                let start = std::time::Instant::now();

                for collider in colliders.iter() {
                    let _ = main_collider.gjk(collider);
                }

                let elapsed = start.elapsed();
                let fps = 1.0 / elapsed.as_secs_f32();
                avg_elapsed += elapsed;
                avg_fps += fps;
                println!("OBB vs OBB (GJK): {:<8.2?} (fps: {})", elapsed, fps);
            }
            avg_elapsed /= LOOP_COUNT as u32;
            avg_fps /= LOOP_COUNT as f32;
            println!("OBB vs OBB (GJK) avg: {:<8.2?} (fps: {})", avg_elapsed, avg_fps);
        }

        {
            let mut avg_elapsed = std::time::Duration::new(0, 0);
            let mut avg_fps = 0.0;
            for _ in 0..LOOP_COUNT {
                let start = std::time::Instant::now();

                for collider in colliders.iter() {
                    let _ = main_collider.obb_collision_details(collider);
                }

                let elapsed = start.elapsed();
                let fps = 1.0 / elapsed.as_secs_f32();
                avg_elapsed += elapsed;
                avg_fps += fps;
                println!("OBB vs OBB (SAT-with normal): {:<8.2?} (fps: {})", elapsed, fps);
            }
            avg_elapsed /= LOOP_COUNT as u32;
            avg_fps /= LOOP_COUNT as f32;
            println!("OBB vs OBB (SAT-with normal) avg: {:<8.2?} (fps: {})", avg_elapsed, avg_fps);
        }

        {
            let mut avg_elapsed = std::time::Duration::new(0, 0);
            let mut avg_fps = 0.0;
            for _ in 0..LOOP_COUNT {
                let start = std::time::Instant::now();

                for collider in colliders.iter() {
                    let _ = main_collider.gjk_epa(collider);
                }

                let elapsed = start.elapsed();
                let fps = 1.0 / elapsed.as_secs_f32();
                avg_elapsed += elapsed;
                avg_fps += fps;
                println!("OBB vs OBB (GJK-EPA): {:<8.2?} (fps: {})", elapsed, fps);
            }
            avg_elapsed /= LOOP_COUNT as u32;
            avg_fps /= LOOP_COUNT as f32;
            println!("OBB vs OBB (GJK-EPA) avg: {:<8.2?} (fps: {})", avg_elapsed, avg_fps);
        }
    }

    println!();

    // 3. OBB vs Sphere
    {
        let main_collider = Sphere {
            center: glam::Vec3::new(0.0, 0.0, 0.0),
            radius: 1.0,
        };

        let mut colliders = Vec::new();
        for _ in 0..1_000_000 {
            let x = rand::random::<f32>() * 100.0 - 50.0;
            let y = rand::random::<f32>() * 100.0 - 50.0;
            let z = rand::random::<f32>() * 100.0 - 50.0;
            let ex = rand::random::<f32>() * 5.0 + 1.0;
            let ey = rand::random::<f32>() * 5.0 + 1.0;
            let ez = rand::random::<f32>() * 5.0 + 1.0;
            let rx = rand::random::<f32>() * 360.0;
            let ry = rand::random::<f32>() * 360.0;
            let rz = rand::random::<f32>() * 360.0;
            let collider = BoundingBox::new_rotated(
                glam::Vec3::new(x, y, z),
                glam::Vec3::new(ex, ey, ez),
                glam::Mat3::from_euler(glam::EulerRot::YXZ, rx.to_radians(), ry.to_radians(), rz.to_radians()),
            );
            colliders.push(collider);
        }

        {
            let mut avg_elapsed = std::time::Duration::new(0, 0);
            let mut avg_fps = 0.0;
            for _ in 0..LOOP_COUNT {
                let start = std::time::Instant::now();

                for collider in colliders.iter() {
                    let _ = main_collider.check_static_collision(collider);
                }

                let elapsed = start.elapsed();
                let fps = 1.0 / elapsed.as_secs_f32();
                avg_elapsed += elapsed;
                avg_fps += fps;
                println!("OBB vs Sphere (SAT): {:<8.2?} (fps: {})", elapsed, fps);
            }
            avg_elapsed /= LOOP_COUNT as u32;
            avg_fps /= LOOP_COUNT as f32;
            println!("OBB vs Sphere (SAT) avg: {:<8.2?} (fps: {})", avg_elapsed, avg_fps);
        }

        {
            let mut avg_elapsed = std::time::Duration::new(0, 0);
            let mut avg_fps = 0.0;
            for _ in 0..LOOP_COUNT {
                let start = std::time::Instant::now();

                for collider in colliders.iter() {
                    let _ = main_collider.gjk(collider);
                }

                let elapsed = start.elapsed();
                let fps = 1.0 / elapsed.as_secs_f32();
                avg_elapsed += elapsed;
                avg_fps += fps;
                println!("OBB vs Sphere (GJK): {:<8.2?} (fps: {})", elapsed, fps);
            }
            avg_elapsed /= LOOP_COUNT as u32;
            avg_fps /= LOOP_COUNT as f32;
            println!("OBB vs Sphere (GJK) avg: {:<8.2?} (fps: {})", avg_elapsed, avg_fps);
        }

        {
            let mut avg_elapsed = std::time::Duration::new(0, 0);
            let mut avg_fps = 0.0;
            for _ in 0..LOOP_COUNT {
                let start = std::time::Instant::now();

                for collider in colliders.iter() {
                    let _ = main_collider.check_static_collision_details(collider);
                }

                let elapsed = start.elapsed();
                let fps = 1.0 / elapsed.as_secs_f32();
                avg_elapsed += elapsed;
                avg_fps += fps;
                println!("OBB vs Sphere (SAT-with normal): {:<8.2?} (fps: {})", elapsed, fps);
            }
            avg_elapsed /= LOOP_COUNT as u32;
            avg_fps /= LOOP_COUNT as f32;
            println!("OBB vs Sphere (SAT-with normal) avg: {:<8.2?} (fps: {})", avg_elapsed, avg_fps);
        }

        {
            let mut avg_elapsed = std::time::Duration::new(0, 0);
            let mut avg_fps = 0.0;
            for _ in 0..LOOP_COUNT {
                let start = std::time::Instant::now();

                for collider in colliders.iter() {
                    let _ = main_collider.gjk_epa(collider);
                }

                let elapsed = start.elapsed();
                let fps = 1.0 / elapsed.as_secs_f32();
                avg_elapsed += elapsed;
                avg_fps += fps;
                println!("OBB vs Sphere (GJK-EPA): {:<8.2?} (fps: {})", elapsed, fps);
            }
            avg_elapsed /= LOOP_COUNT as u32;
            avg_fps /= LOOP_COUNT as f32;
            println!("OBB vs Sphere (GJK-EPA) avg: {:<8.2?} (fps: {})", avg_elapsed, avg_fps);
        }
    }
}
