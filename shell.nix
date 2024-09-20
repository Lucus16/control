{ pkgs ? import <nixpkgs> {} }:

let
  libraries = with pkgs; [
    alsaLib
    libglvnd
    udev
    xorg.libXcursor
    xorg.libXi
    xorg.libXrandr
    xorg.libxcb
  ];
in pkgs.mkShell {
  nativeBuildInputs = with pkgs; [
    cargo
    cargo-watch
    cmake
    pkg-config
    rustfmt
  ];

  buildInputs = libraries;
  LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath libraries;
}
