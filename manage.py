#!/usr/bin/env python3
"""
Cross-platform build and management script for city-CBS-Astar project.

This script provides a unified interface for building, running, and managing
the project across Linux, macOS, and Windows platforms.

Usage:
    python manage.py [command] [options]

Commands:
    build       Build the project (debug or release)
    run         Run the built executable
    clean       Clean build artifacts
    doc         Generate documentation
    sign        Sign the binary (macOS only)
    help        Show this help message

Examples:
    python manage.py build --debug
    python manage.py build --release
    python manage.py run -- data 1 10 5
    python manage.py run -- run 5
    python manage.py clean
"""

import sys
import os
import subprocess
import shutil
import platform
import argparse
from pathlib import Path


class ProjectManager:
    """Manage build, run, and maintenance tasks for the project."""
    
    def __init__(self):
        self.project_root = Path(__file__).parent.absolute()
        self.build_dir = self.project_root / "build"
        self.system = platform.system()
        self.executable_name = "city-cbs-astar"
        if self.system == "Windows":
            self.executable_name += ".exe"
        self.executable_path = self.build_dir / "bin" / self.executable_name
        
    def build(self, build_type="Release", jobs=None):
        """Build the project with specified build type."""
        print(f"Building project in {build_type} mode...")
        
        # Determine number of jobs
        if jobs is None:
            try:
                jobs = os.cpu_count() or 4
            except:
                jobs = 4
        
        # Configure CMake
        cmake_args = [
            "cmake",
            "-DCMAKE_BUILD_TYPE=" + build_type,
            "-B", str(self.build_dir)
        ]
        
        # Add generator for Windows if needed
        if self.system == "Windows":
            cmake_args.extend(["-G", "Ninja"])
        
        try:
            subprocess.run(cmake_args, cwd=self.project_root, check=True)
        except subprocess.CalledProcessError as e:
            print(f"CMake configuration failed: {e}")
            return False
        
        # Build
        build_args = [
            "cmake",
            "--build", str(self.build_dir),
            "-j", str(jobs)
        ]
        
        try:
            subprocess.run(build_args, cwd=self.project_root, check=True)
            print(f"Build completed successfully!")
            return True
        except subprocess.CalledProcessError as e:
            print(f"Build failed: {e}")
            return False
    
    def run(self, args):
        """Run the executable with provided arguments."""
        if not self.executable_path.exists():
            print(f"Executable not found at {self.executable_path}")
            print("Please build the project first using: python manage.py build")
            return False
        
        cmd = [str(self.executable_path)] + args
        print(f"Running: {' '.join(cmd)}")
        
        try:
            subprocess.run(cmd, cwd=self.project_root, check=True)
            return True
        except subprocess.CalledProcessError as e:
            print(f"Execution failed: {e}")
            return False
        except KeyboardInterrupt:
            print("\nExecution interrupted by user")
            return False
    
    def clean(self):
        """Clean build artifacts."""
        print("Cleaning build artifacts...")
        
        # Remove build directory
        if self.build_dir.exists():
            shutil.rmtree(self.build_dir)
            print(f"Removed {self.build_dir}")
        
        # Remove LaTeX documentation directory
        latex_dir = self.project_root / "doc" / "latex"
        if latex_dir.exists():
            shutil.rmtree(latex_dir)
            print(f"Removed {latex_dir}")
        
        print("Clean completed!")
        return True
    
    def doc(self):
        """Generate documentation using Doxygen and LaTeX."""
        print("Generating documentation...")
        print("Warning: Create the LaTeX files using Doxygen first")
        
        latex_dir = self.project_root / "doc" / "latex"
        if not latex_dir.exists():
            print("Error: The LaTeX files are not created")
            print("Please run Doxygen first to generate the LaTeX files")
            return False
        
        try:
            subprocess.run(["make"], cwd=latex_dir, check=True)
            
            # Copy the PDF to documentation.pdf
            refman_pdf = latex_dir / "refman.pdf"
            doc_pdf = self.project_root / "doc" / "documentation.pdf"
            shutil.copy(refman_pdf, doc_pdf)
            
            print(f"Documentation generated at {doc_pdf}")
            return True
        except subprocess.CalledProcessError as e:
            print(f"Documentation generation failed: {e}")
            return False
    
    def sign(self):
        """Sign the binary for macOS."""
        if self.system != "Darwin":
            print("Code signing is only supported on macOS")
            return False
        
        if not self.executable_path.exists():
            print(f"Executable not found at {self.executable_path}")
            print("Please build the project first")
            return False
        
        entitlements = self.project_root / "Entitlements.plist"
        if not entitlements.exists():
            print(f"Entitlements file not found at {entitlements}")
            return False
        
        try:
            subprocess.run([
                "codesign",
                "--entitlements", str(entitlements),
                "-s", "-",
                str(self.executable_path)
            ], check=True)
            print("Code signing completed!")
            return True
        except subprocess.CalledProcessError as e:
            print(f"Code signing failed: {e}")
            return False


def main():
    """Main entry point for the script."""
    parser = argparse.ArgumentParser(
        description="Cross-platform build and management script for city-CBS-Astar",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python manage.py build --debug
  python manage.py build --release
  python manage.py run -- data 1 10 5
  python manage.py run -- run 5
  python manage.py clean
        """
    )
    
    subparsers = parser.add_subparsers(dest="command", help="Command to execute")
    
    # Build command
    build_parser = subparsers.add_parser("build", help="Build the project")
    build_group = build_parser.add_mutually_exclusive_group()
    build_group.add_argument("--debug", action="store_true", help="Build in debug mode")
    build_group.add_argument("--release", action="store_true", help="Build in release mode (default)")
    build_parser.add_argument("-j", "--jobs", type=int, help="Number of parallel build jobs")
    
    # Run command
    run_parser = subparsers.add_parser("run", help="Run the executable")
    run_parser.add_argument("args", nargs="*", help="Arguments to pass to the executable")
    
    # Clean command
    subparsers.add_parser("clean", help="Clean build artifacts")
    
    # Doc command
    subparsers.add_parser("doc", help="Generate documentation")
    
    # Sign command
    subparsers.add_parser("sign", help="Sign the binary (macOS only)")
    
    # Help command
    subparsers.add_parser("help", help="Show this help message")
    
    args = parser.parse_args()
    
    if not args.command or args.command == "help":
        parser.print_help()
        return 0
    
    manager = ProjectManager()
    
    if args.command == "build":
        build_type = "Debug" if args.debug else "Release"
        success = manager.build(build_type, args.jobs)
    elif args.command == "run":
        success = manager.run(args.args)
    elif args.command == "clean":
        success = manager.clean()
    elif args.command == "doc":
        success = manager.doc()
    elif args.command == "sign":
        success = manager.sign()
    else:
        parser.print_help()
        return 1
    
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
