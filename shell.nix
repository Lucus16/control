{ pkgs ? import <nixpkgs> {} }:

with pkgs;

let
  libraries = lib.makeLibraryPath [
    # vulkan-loader
    xorg.libXcursor
    xorg.libXrandr
    xorg.libXi
    xlibs.libX11
    libglvnd
  ];

in mkShell {
  buildInputs = [ cargo cargo-watch cmake rustfmt ];
  shellHook = ''
    export LD_LIBRARY_PATH=${libraries}
  '';
}
