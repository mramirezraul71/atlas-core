"""
ATLAS NEXUS - Migration Script
Integrates existing ATLAS_PUSH project with ATLAS NEXUS
"""

import os
import shutil
from pathlib import Path
import json

class AtlasMigration:
    """
    Migrate existing ATLAS_PUSH to ATLAS NEXUS
    """
    
    def __init__(self, old_project_path: str, new_project_path: str):
        self.old_path = Path(old_project_path)
        self.new_path = Path(new_project_path)
        
        print("=" * 60)
        print("  ATLAS NEXUS - Migration Wizard")
        print("=" * 60)
        print()
    
    def run(self):
        """Execute migration"""
        print("🔄 Starting migration...")
        print()
        
        # Step 1: Backup old project
        print("[1/6] Creating backup...")
        self.backup_old_project()
        
        # Step 2: Copy modules
        print("[2/6] Migrating modules...")
        self.migrate_modules()
        
        # Step 3: Migrate configuration
        print("[3/6] Migrating configuration...")
        self.migrate_config()
        
        # Step 4: Migrate data
        print("[4/6] Migrating data (memory, logs, snapshots)...")
        self.migrate_data()
        
        # Step 5: Create integration layer
        print("[5/6] Creating integration layer...")
        self.create_integration()
        
        # Step 6: Summary
        print("[6/6] Finalizing...")
        self.print_summary()
        
        print()
        print("✅ Migration complete!")
        print()
    
    def backup_old_project(self):
        """Backup existing project"""
        backup_dir = self.old_path.parent / f"ATLAS_PUSH_backup_{Path(__file__).stem}"
        
        if not backup_dir.exists():
            shutil.copytree(self.old_path, backup_dir)
            print(f"  ✓ Backup created: {backup_dir}")
        else:
            print(f"  ! Backup already exists: {backup_dir}")
    
    def migrate_modules(self):
        """Migrate custom modules"""
        # Copy telegram module
        old_telegram = self.old_path / "modules" / "atlas_telegram.py"
        new_modules = self.new_path / "modules"
        new_modules.mkdir(exist_ok=True)
        
        if old_telegram.exists():
            shutil.copy2(old_telegram, new_modules / "atlas_telegram.py")
            print(f"  ✓ Migrated: atlas_telegram.py")
        
        # Copy other modules
        modules_to_migrate = [
            "atlas_chat.py",
            "atlas_gui.py",
            "atlas_telegram_bot.py",
            "command_router.py",
            "rauli_doctor.py"
        ]
        
        for module in modules_to_migrate:
            old_module = self.old_path / "modules" / module
            if old_module.exists():
                shutil.copy2(old_module, new_modules / module)
                print(f"  ✓ Migrated: {module}")
    
    def migrate_config(self):
        """Migrate configuration"""
        # Read old config
        old_config = self.old_path / "config" / "atlas.env"
        new_config = self.new_path / ".env"
        
        if old_config.exists() and not new_config.exists():
            shutil.copy2(old_config, new_config)
            print(f"  ✓ Migrated configuration")
        else:
            print(f"  ! Configuration already exists or not found")
        
        # Copy to C:\ATLAS\config if exists
        atlas_config = Path("C:/ATLAS/config/.env")
        if atlas_config.parent.exists() and old_config.exists():
            shutil.copy2(old_config, atlas_config)
            print(f"  ✓ Configuration copied to C:\\ATLAS\\config")
    
    def migrate_data(self):
        """Migrate data directories"""
        data_dirs = {
            'memory': 'memory/atlas_memory.txt',
            'logs': 'logs',
            'snapshots': 'snapshots',
            'ATLAS_VAULT': 'ATLAS_VAULT'
        }
        
        # Check if C:\ATLAS exists
        atlas_root = Path("C:/ATLAS")
        if not atlas_root.exists():
            atlas_root.mkdir(parents=True)
            print(f"  ✓ Created C:\\ATLAS directory")
        
        for dir_name, source in data_dirs.items():
            old_dir = self.old_path / source if '/' not in source else self.old_path / source.split('/')[0]
            new_dir = atlas_root / dir_name
            
            if old_dir.exists() and not new_dir.exists():
                shutil.copytree(old_dir, new_dir)
                print(f"  ✓ Migrated: {dir_name}")
            elif old_dir.exists():
                print(f"  ! {dir_name} already exists, skipping")
    
    def create_integration(self):
        """Create integration layer for backward compatibility"""
        integration_code = '''"""
ATLAS NEXUS - Legacy Integration
Provides backward compatibility with ATLAS_PUSH modules
"""

import sys
from pathlib import Path

# Add ATLAS_PUSH modules to path for compatibility
legacy_modules_path = Path(__file__).parent / "modules"
if str(legacy_modules_path) not in sys.path:
    sys.path.insert(0, str(legacy_modules_path))

# Re-export commonly used functions for backward compatibility
try:
    from modules.command_router import handle as legacy_handle
    from modules.atlas_telegram import main as telegram_main
except ImportError:
    legacy_handle = None
    telegram_main = None

def get_legacy_handler():
    """Get legacy command handler"""
    return legacy_handle

def start_telegram():
    """Start legacy Telegram bot"""
    if telegram_main:
        return telegram_main()
    else:
        print("Telegram module not available")
'''
        
        integration_file = self.new_path / "legacy_integration.py"
        integration_file.write_text(integration_code)
        print(f"  ✓ Created legacy integration layer")
    
    def print_summary(self):
        """Print migration summary"""
        summary = f"""
╔══════════════════════════════════════════════════════════════╗
║                  MIGRATION SUMMARY                           ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  ✓ Old project backed up                                    ║
║  ✓ Modules migrated                                         ║
║  ✓ Configuration migrated                                   ║
║  ✓ Data migrated (memory, logs, snapshots)                  ║
║  ✓ Integration layer created                                ║
║                                                              ║
╠══════════════════════════════════════════════════════════════╣
║                  NEXT STEPS                                  ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  1. Review migrated configuration:                          ║
║     {str(self.new_path / '.env'):<50}║
║                                                              ║
║  2. Test ATLAS NEXUS:                                       ║
║     python atlas_nexus.py                                   ║
║                                                              ║
║  3. Test Telegram bot:                                      ║
║     python atlas_nexus.py telegram                          ║
║                                                              ║
║  4. Start API server:                                       ║
║     python atlas_nexus.py api                               ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
"""
        print(summary)


def main():
    """Main migration entry point"""
    import sys
    
    if len(sys.argv) < 3:
        print("Usage: python migrate.py <old_project_path> <new_project_path>")
        print("Example: python migrate.py C:/ATLAS_PUSH C:/atlas_nexus")
        sys.exit(1)
    
    old_path = sys.argv[1]
    new_path = sys.argv[2]
    
    if not Path(old_path).exists():
        print(f"Error: Old project path not found: {old_path}")
        sys.exit(1)
    
    if not Path(new_path).exists():
        print(f"Error: New project path not found: {new_path}")
        sys.exit(1)
    
    migration = AtlasMigration(old_path, new_path)
    migration.run()


if __name__ == "__main__":
    main()
