#!/usr/bin/env python3
"""
Atlas Git Error Detector
Detecta errores de Git y aplica protocolo de manejo automáticamente
"""

import subprocess
import re
import json
import sys
from datetime import datetime
from pathlib import Path

class GitErrorDetector:
    """Detector de errores de Git con protocolo automático"""
    
    def __init__(self):
        self.error_patterns = {
            'repository_rules': r'Repository rule violations found',
            'push_declined': r'push declined',
            'authentication_failed': r'Authentication failed',
            'permission_denied': r'Permission denied',
            'secrets_detected': r'Push cannot contain secrets',
            'force_required': r'force-push required',
            'merge_conflict': r'merge conflict',
            'detached_head': r'detached HEAD',
            'remote_disconnected': r'remote: disconnected'
        }
        
        self.critical_errors = [
            'repository_rules',
            'secrets_detected',
            'permission_denied',
            'authentication_failed'
        ]
        
        self.log_file = Path("logs/git_error_detector.log")
        self.log_file.parent.mkdir(exist_ok=True)
        
    def log_error(self, error_type: str, message: str, command: str):
        """Registra error en log"""
        timestamp = datetime.now().isoformat()
        log_entry = {
            "timestamp": timestamp,
            "error_type": error_type,
            "message": message,
            "command": command,
            "action_required": "CALL_SPECIALIST" if error_type in self.critical_errors else "REVIEW_MANUAL"
        }
        
        with open(self.log_file, 'a') as f:
            f.write(json.dumps(log_entry, indent=2) + "\n")
            
        return log_entry
    
    def detect_error_type(self, error_output: str) -> str:
        """Detecta tipo de error basado en el output"""
        for error_type, pattern in self.error_patterns.items():
            if re.search(pattern, error_output, re.IGNORECASE):
                return error_type
        return "unknown"
    
    def run_git_command(self, command: str) -> dict:
        """Ejecuta comando Git y detecta errores"""
        try:
            result = subprocess.run(
                command.split(),
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode != 0:
                error_type = self.detect_error_type(result.stderr)
                log_entry = self.log_error(error_type, result.stderr, command)
                
                return {
                    "success": False,
                    "error_type": error_type,
                    "error_message": result.stderr,
                    "log_entry": log_entry,
                    "requires_specialist": error_type in self.critical_errors
                }
            
            return {
                "success": True,
                "output": result.stdout,
                "error_type": None
            }
            
        except subprocess.TimeoutExpired:
            error_msg = "Git command timed out"
            log_entry = self.log_error("timeout", error_msg, command)
            return {
                "success": False,
                "error_type": "timeout",
                "error_message": error_msg,
                "log_entry": log_entry,
                "requires_specialist": True
            }
        except Exception as e:
            error_msg = str(e)
            log_entry = self.log_error("system_error", error_msg, command)
            return {
                "success": False,
                "error_type": "system_error",
                "error_message": error_msg,
                "log_entry": log_entry,
                "requires_specialist": True
            }
    
    def check_pre_push_safety(self) -> dict:
        """Verificación de seguridad antes de push"""
        safety_checks = {
            "sensitive_files": self._check_sensitive_files(),
            "gitignore_status": self._check_gitignore(),
            "current_branch": self._get_current_branch(),
            "uncommitted_changes": self._check_uncommitted_changes(),
            "large_files": self._check_large_files()
        }
        
        return {
            "safe": all(check.get("safe", True) for check in safety_checks.values()),
            "checks": safety_checks
        }
    
    def _check_sensitive_files(self) -> dict:
        """Verifica archivos sensibles en staging"""
        try:
            result = subprocess.run(
                ["git", "diff", "--cached", "--name-only"],
                capture_output=True,
                text=True
            )
            
            if result.returncode != 0:
                return {"safe": False, "error": "Cannot check staged files"}
            
            staged_files = result.stdout.strip().split('\n') if result.stdout.strip() else []
            sensitive_patterns = ['.key', '.pem', '.env', '.secret', '.token']
            
            sensitive_files = []
            for file_path in staged_files:
                for pattern in sensitive_patterns:
                    if pattern in file_path:
                        sensitive_files.append(file_path)
                        break
            
            return {
                "safe": len(sensitive_files) == 0,
                "sensitive_files": sensitive_files,
                "total_staged": len(staged_files)
            }
            
        except Exception as e:
            return {"safe": False, "error": str(e)}
    
    def _check_gitignore(self) -> dict:
        """Verifica estado de .gitignore"""
        gitignore_path = Path(".gitignore")
        
        if not gitignore_path.exists():
            return {"safe": False, "error": ".gitignore not found"}
        
        try:
            with open(gitignore_path, 'r') as f:
                content = f.read()
            
            required_patterns = ['.key', '.pem', '.env', '*.secret', '*.token']
            missing_patterns = [p for p in required_patterns if p not in content]
            
            return {
                "safe": len(missing_patterns) == 0,
                "missing_patterns": missing_patterns,
                "gitignore_exists": True
            }
            
        except Exception as e:
            return {"safe": False, "error": str(e)}
    
    def _get_current_branch(self) -> dict:
        """Obtiene branch actual"""
        try:
            result = subprocess.run(
                ["git", "branch", "--show-current"],
                capture_output=True,
                text=True
            )
            
            if result.returncode != 0:
                return {"safe": False, "error": "Cannot get current branch"}
            
            branch = result.stdout.strip()
            protected_branches = ['main', 'master', 'develop']
            
            return {
                "safe": branch not in protected_branches,
                "current_branch": branch,
                "is_protected": branch in protected_branches
            }
            
        except Exception as e:
            return {"safe": False, "error": str(e)}
    
    def _check_uncommitted_changes(self) -> dict:
        """Verifica cambios sin commitear"""
        try:
            result = subprocess.run(
                ["git", "status", "--porcelain"],
                capture_output=True,
                text=True
            )
            
            if result.returncode != 0:
                return {"safe": False, "error": "Cannot check git status"}
            
            changes = result.stdout.strip().split('\n') if result.stdout.strip() else []
            uncommitted = [line for line in changes if line.strip()]
            
            return {
                "safe": len(uncommitted) == 0,
                "uncommitted_count": len(uncommitted),
                "changes": uncommitted[:10]  # Primeros 10 cambios
            }
            
        except Exception as e:
            return {"safe": False, "error": str(e)}
    
    def _check_large_files(self) -> dict:
        """Verifica archivos grandes en staging"""
        try:
            result = subprocess.run(
                ["git", "diff", "--cached", "--name-only"],
                capture_output=True,
                text=True
            )
            
            if result.returncode != 0:
                return {"safe": False, "error": "Cannot check staged files"}
            
            staged_files = result.stdout.strip().split('\n') if result.stdout.strip() else []
            large_files = []
            
            for file_path in staged_files:
                try:
                    file_size = Path(file_path).stat().st_size if Path(file_path).exists() else 0
                    if file_size > 10 * 1024 * 1024:  # 10MB
                        large_files.append({
                            "file": file_path,
                            "size_mb": file_size / (1024 * 1024)
                        })
                except:
                    pass
            
            return {
                "safe": len(large_files) == 0,
                "large_files": large_files,
                "max_size_mb": 10
            }
            
        except Exception as e:
            return {"safe": False, "error": str(e)}
    
    def generate_error_report(self, error_result: dict) -> str:
        """Genera reporte de error para especialista"""
        if error_result["success"]:
            return "No error detected"
        
        report = f"""
## 🚨 GIT ERROR REPORT

**Timestamp:** {datetime.now().isoformat()}
**Error Type:** {error_result['error_type']}
**Requires Specialist:** {'YES' if error_result['requires_specialist'] else 'NO'}

### 🔍 Error Details:
```
{error_result['error_message']}
```

### 📋 Recommended Action:
"""
        
        if error_result['requires_specialist']:
            report += """
**🚨 CRITICAL - CALL SPECIALIST IMMEDIATELY**

1. STOP all git operations
2. DO NOT use --force
3. Contact Git specialist
4. Wait for resolution
5. Document the resolution

**Specialist Contact:**
- DevOps Team: devops@atlas.ai
- GitHub Admin: admin@atlas.ai
- Emergency: emergency@atlas.ai
"""
        else:
            report += """
**⚠️ REVIEW REQUIRED**

1. Review the error message
2. Check git status
3. Resolve conflicts locally
4. Try again without --force
5. Contact specialist if persists
"""
        
        return report
    
    def safe_push(self, remote: str = "origin", branch: str = None) -> dict:
        """Push seguro con verificación previa"""
        if not branch:
            # Obtener branch actual
            try:
                result = subprocess.run(
                    ["git", "branch", "--show-current"],
                    capture_output=True,
                    text=True
                )
                branch = result.stdout.strip()
            except:
                return {"success": False, "error": "Cannot determine current branch"}
        
        # 1. Verificación de seguridad
        safety_check = self.check_pre_push_safety()
        
        if not safety_check["safe"]:
            return {
                "success": False,
                "error": "Safety check failed",
                "safety_check": safety_check,
                "requires_specialist": True
            }
        
        # 2. Ejecutar push con detección de errores
        push_command = f"git push {remote} {branch}"
        result = self.run_git_command(push_command)
        
        if not result["success"] and result["requires_specialist"]:
            # Generar reporte para especialista
            report = self.generate_error_report(result)
            
            # Guardar reporte
            report_file = Path(f"git_error_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.md")
            with open(report_file, 'w') as f:
                f.write(report)
            
            result["report_file"] = str(report_file)
            result["report_content"] = report
        
        return result

def main():
    """Función principal para uso desde línea de comandos"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Atlas Git Error Detector")
    parser.add_argument("--check-safety", action="store_true", help="Check pre-push safety")
    parser.add_argument("--safe-push", help="Safe push to remote", default=None)
    parser.add_argument("--command", help="Run git command with error detection")
    
    args = parser.parse_args()
    
    detector = GitErrorDetector()
    
    if args.check_safety:
        safety = detector.check_pre_push_safety()
        print("🔍 Safety Check Results:")
        print(json.dumps(safety, indent=2))
        
        if not safety["safe"]:
            print("\n🚨 SAFETY ISSUES DETECTED - DO NOT PUSH")
            sys.exit(1)
        else:
            print("\n✅ Safety check passed")
            sys.exit(0)
    
    elif args.safe_push:
        result = detector.safe_push(branch=args.safe_push)
        
        if result["success"]:
            print(f"✅ Push successful to {args.safe_push}")
        else:
            print(f"❌ Push failed: {result.get('error', 'Unknown error')}")
            
            if result.get("requires_specialist"):
                print("\n🚨 CRITICAL ERROR - CALL SPECIALIST")
                if "report_file" in result:
                    print(f"📄 Report saved to: {result['report_file']}")
            
            sys.exit(1)
    
    elif args.command:
        result = detector.run_git_command(args.command)
        
        if result["success"]:
            print(f"✅ Command successful: {args.command}")
        else:
            print(f"❌ Command failed: {args.command}")
            print(f"Error type: {result['error_type']}")
            
            if result["requires_specialist"]:
                print("\n🚨 CRITICAL ERROR - CALL SPECIALIST")
            
            sys.exit(1)
    
    else:
        print("Atlas Git Error Detector")
        print("Use --help for available options")

if __name__ == "__main__":
    main()
