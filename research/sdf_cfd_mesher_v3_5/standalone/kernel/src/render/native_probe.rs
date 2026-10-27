//! Optional automated timing in the real App. No alternate window/application.
use super::{Camera, telemetry};
use eframe::egui;
use std::time::Instant;

pub struct NativeProbe {
    start: Instant,
    last: Instant,
    requested: bool,
    captured: bool,
    frames: Vec<f64>,
    pub round: usize,
    session_start: Instant,
    finished: bool,
    settled_frames: Vec<f64>,
    capture_view: usize,
    capture_wait: u32,
}
impl NativeProbe {
    pub fn new() -> Self {
        Self {
            start: Instant::now(),
            last: Instant::now(),
            requested: false,
            captured: false,
            frames: Vec::new(),
            round: 0,
            session_start: Instant::now(),
            finished: false,
            settled_frames: Vec::new(),
            capture_view: 0,
            capture_wait: 0,
        }
    }
    pub fn tick(
        &mut self,
        ctx: &egui::Context,
        ready: bool,
        camera: &mut Camera,
        revision: u64,
        conditioned_ready: bool,
    ) -> bool {
        if let Ok(folder)=std::env::var("IMPLICIT_VIEWPORT_CAPTURE_DIR") {
            self.capture_views(ctx,camera,conditioned_ready,&folder);
            return false;
        }
        // Manual native input qualification: retain isolated probe loading/no autosave,
        // but do not inject orbit or close the window during controlled gestures.
        if std::env::var_os("IMPLICIT_CAMERA_DIAGNOSTICS").is_some() {
            return false;
        }
        if self.finished {
            return false;
        }
        let scheduling_mode = std::env::var("IMPLICIT_CONDITIONED_PROBE").ok();
        let now = Instant::now();
        let dt = now.duration_since(self.last).as_secs_f64() * 1000.0;
        self.last = now;
        if ready && !self.requested {
            self.requested = true;
            ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot);
        }
        let screenshot = ctx.input(|i| {
            i.events
                .iter()
                .any(|e| matches!(e, egui::Event::Screenshot { .. }))
        });
        if screenshot && !self.captured {
            self.captured = true;
            telemetry::record(
                "update_to_screenshot_delivered_upper_bound",
                self.start.elapsed().as_secs_f64() * 1000.0,
            );
            telemetry::lifecycle("canonical_screenshot_delivered", revision);
        }
        if scheduling_mode.is_some() {
            self.frames.push(dt);
            camera.orbit(0.5, 0.1);
            let rapid = scheduling_mode.as_deref() == Some("rapid")
                || scheduling_mode.as_deref() == Some("replace")
                || scheduling_mode.as_deref() == Some("grid_race")
                || scheduling_mode.as_deref() == Some("gpu_grid_race")
                || scheduling_mode.as_deref() == Some("cancel_burst")
                || scheduling_mode.as_deref() == Some("rect_modes");
            let edit_count=if scheduling_mode.as_deref()==Some("rect_modes"){6}else if scheduling_mode.as_deref()==Some("cancel_burst") {8}else{2};
            let edit_ms = if scheduling_mode.as_deref() == Some("grid_race") {
                12000 + self.round as u128 * 3000
            } else if scheduling_mode.as_deref()==Some("cancel_burst") {
                (self.round as u128+1)*750
            } else {
                (self.round as u128 + 1) * 1500
            };
            let edit_due = if scheduling_mode.as_deref() == Some("rect_modes") {conditioned_ready && self.captured} else if scheduling_mode.as_deref() == Some("gpu_grid_race") {
                if self.round == 0 {
                    telemetry::latest_conditioned_grid_started() == revision
                } else {
                    self.start.elapsed().as_millis() >= 1500
                }
            } else {
                self.session_start.elapsed().as_millis() >= edit_ms
            };
            if rapid && self.round < edit_count && edit_due {
                self.round += 1;
                self.start = Instant::now();
                self.requested = false;
                self.captured = false;
                telemetry::lifecycle("probe_edit_requested", revision + 1);
                return true;
            }
            let last_round = if rapid { edit_count } else { 0 };
            if conditioned_ready && self.round == last_round && self.captured {
                let samples = std::env::var("IMPLICIT_VIEWPORT_SETTLED_FRAMES").ok()
                    .and_then(|s| s.parse::<usize>().ok()).unwrap_or(0);
                if samples == 0 {
                    self.finish(ctx);
                } else {
                    self.settled_frames.push(dt);
                    if self.settled_frames.len() >= samples + 30 {
                        let mut times=self.settled_frames[30..].to_vec();
                        times.sort_by(f64::total_cmp);
                        telemetry::record("settled_frame_median",times[times.len()/2]);
                        telemetry::record("settled_frame_p95",times[times.len()*95/100]);
                        telemetry::record("settled_frame_max",*times.last().unwrap());
                        self.finish(ctx);
                    }
                }
            } else if self.session_start.elapsed().as_secs() > 150 {
                eprintln!("VIEWPORT_PROBE_TIMEOUT");
                self.finish(ctx);
            }
            ctx.request_repaint();
            return false;
        }
        if self.captured {
            self.frames.push(dt);
            camera.orbit(0.5, 0.1);
            let frame_limit = std::env::var("IMPLICIT_VIEWPORT_PROBE_FRAMES")
                .ok()
                .and_then(|s| s.parse::<usize>().ok())
                .unwrap_or(180)
                .max(31);
            if self.frames.len() == frame_limit {
                let mut samples = self.frames[30..].to_vec();
                samples.sort_by(f64::total_cmp);
                telemetry::record("orbit_frame_interval_median", samples[samples.len() / 2]);
                telemetry::record(
                    "orbit_frame_interval_p95",
                    samples[samples.len() * 95 / 100],
                );
                telemetry::record("orbit_frame_interval_max", *samples.last().unwrap());
                eprintln!("VIEWPORT_ROUND {}", self.round);
                telemetry::flush();
                let rounds = std::env::var("IMPLICIT_VIEWPORT_PROBE_ROUNDS")
                    .ok()
                    .and_then(|s| s.parse::<usize>().ok())
                    .unwrap_or(3)
                    .max(1);
                if self.round + 1 < rounds {
                    let next = self.round + 1;
                    *self = Self::new();
                    self.round = next;
                    return true;
                } else {
                    self.finished = true;
                    ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                }
            }
        }
        if self.start.elapsed().as_secs() > 90 {
            eprintln!("VIEWPORT_PROBE_TIMEOUT");
            telemetry::flush();
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
        ctx.request_repaint();
        false
    }

    // Deterministic screenshot fixture only; does not modify Camera input behavior.
    fn capture_views(&mut self,ctx:&egui::Context,camera:&mut Camera,ready:bool,folder:&str){
        if !ready{ctx.request_repaint();return;}
        let views=[
            ("root_te",glam::Vec3::new(65.,-12.,2.),glam::Vec3::new(60.,0.,0.)),
            ("mid_te",glam::Vec3::new(55.62466,18.,5.),glam::Vec3::new(47.62466,30.,0.)),
            ("tip_te",glam::Vec3::new(38.24932,75.,2.),glam::Vec3::new(35.24932,60.,0.)),
            ("whole",glam::Vec3::new(214.,-324.,180.),glam::Vec3::new(25.,0.,0.)),
            ("leading",glam::Vec3::new(-8.,-12.,3.),glam::Vec3::ZERO),
            ("oblique",glam::Vec3::new(178.,270.,126.),glam::Vec3::new(25.,0.,0.)),
        ];
        if self.capture_view>=views.len(){telemetry::flush();ctx.send_viewport_cmd(egui::ViewportCommand::Close);return;}
        if self.capture_wait>=3 {
            let shot=ctx.input(|i|i.events.iter().find_map(|e|if let egui::Event::Screenshot{image,..}=e{Some(image.clone())}else{None}));
            if let Some(image)=shot{
                let(name,eye,target)=views[self.capture_view];let [w,h]=image.size;let row=(w*3+3)&!3;
                let mut bytes=vec![0u8;54+row*h];bytes[..2].copy_from_slice(b"BM");
                let len=bytes.len()as u32;bytes[2..6].copy_from_slice(&len.to_le_bytes());bytes[10..14].copy_from_slice(&54u32.to_le_bytes());bytes[14..18].copy_from_slice(&40u32.to_le_bytes());
                bytes[18..22].copy_from_slice(&(w as i32).to_le_bytes());bytes[22..26].copy_from_slice(&(h as i32).to_le_bytes());bytes[26..28].copy_from_slice(&1u16.to_le_bytes());bytes[28..30].copy_from_slice(&24u16.to_le_bytes());
                for y in 0..h{for x in 0..w{let c=image.pixels[x+y*w].to_array();let i=54+(h-1-y)*row+x*3;bytes[i..i+3].copy_from_slice(&[c[2],c[1],c[0]]);}}
                let path=std::path::Path::new(folder).join(format!("{name}.bmp"));
                match std::fs::write(&path,bytes){Ok(())=>eprintln!("NATIVE_CAPTURE {}",serde_json::json!({"path":path,"size":[w,h],"eye":eye.to_array(),"target":target.to_array(),"field":"conditioned"})),Err(e)=>eprintln!("NATIVE_CAPTURE_ERROR {e}")}
                self.capture_view+=1;self.capture_wait=0;
            }
        }
        if self.capture_view<views.len(){
            if self.capture_wait==0{camera.eye=views[self.capture_view].1;camera.target=views[self.capture_view].2;camera.up=glam::Vec3::Z;}
            self.capture_wait+=1;
            if self.capture_wait==3{ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot);}
        }
        ctx.request_repaint();
    }

    fn finish(&mut self, ctx: &egui::Context) {
        self.finished = true;
        let mut samples = self.frames.clone();
        samples.sort_by(f64::total_cmp);
        telemetry::record("orbit_frame_interval_median", samples[samples.len() / 2]);
        telemetry::record(
            "orbit_frame_interval_p95",
            samples[samples.len() * 95 / 100],
        );
        telemetry::record("orbit_frame_interval_max", *samples.last().unwrap());
        eprintln!("VIEWPORT_ROUND {}", self.round);
        telemetry::flush();
        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
    }
}
