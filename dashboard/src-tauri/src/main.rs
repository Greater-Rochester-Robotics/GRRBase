#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use tauri::Manager;
use tauri::{utils::config::AppUrl, WindowUrl};
use tauri_plugin_positioner::{WindowExt, Position};

fn main() {
  let port = portpicker::pick_unused_port().expect("failed to find unused port");

  let mut context = tauri::generate_context!();
  let url = format!("http://localhost:{}", port).parse().unwrap();
  let window_url = WindowUrl::External(url);

  context.config_mut().build.dist_dir = AppUrl::Url(window_url.clone());

  tauri::Builder::default()
    .plugin(tauri_plugin_localhost::Builder::new(port).build())
    .setup(|app| {
      let win = app.get_window("main").unwrap();
      let _ = win.move_window(Position::TopCenter);
      Ok(())
    })
    .run(context)
    .expect("error while running tauri application");
}
