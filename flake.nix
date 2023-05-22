{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-22.11";
    flake-utils.url = "github:numtide/flake-utils";

    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs = {
        nixpkgs.follows = "nixpkgs";
        flake-utils.follows = "flake-utils";
      };
    };
    crane = {
      url = "github:ipetkov/crane";
      inputs = {
        nixpkgs.follows = "nixpkgs";
        flake-utils.follows = "flake-utils";
        rust-overlay.follows = "rust-overlay";
      };
    };
  };
  outputs = { self, nixpkgs, flake-utils, rust-overlay, crane }:
    flake-utils.lib.eachDefaultSystem
      (system:
        let
          overlays = [ (import rust-overlay) ];
          pkgs = import nixpkgs {
            inherit system overlays;
          };
          craneLib = crane.lib.${system};
          src = craneLib.cleanCargoSource (craneLib.path ./.);
          nativeBuildInputs = with pkgs; [ rust-bin.stable.latest.default ];
          commonArgs = {
            inherit src nativeBuildInputs;
          };
          cargoArtifacts = craneLib.buildDepsOnly commonArgs;
          rotten-crate = craneLib.buildPackage (commonArgs // { inherit cargoArtifacts; });
        in
        with pkgs;
        {
          checks = {
            inherit rotten-crate;
            fmt = craneLib.cargoFmt { inherit src; };
            tests = craneLib.cargoNextest (commonArgs // {
              inherit cargoArtifacts;
            });
            clippy = craneLib.cargoClippy (commonArgs // {
              inherit cargoArtifacts;
            });
          };
          packages = {
            inherit rotten-crate;
            default = rotten-crate;
          };
          devShells.default = mkShell {
            inputsFrom = [ rotten-crate ];
            buildInputs = [ pkgs.cargo-nextest ];
          };
        }
      );
}

