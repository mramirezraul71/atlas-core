
import os, subprocess
os.environ["QUANT_EXIT_GOVERNANCE_ENABLED"] = "false"
os.environ["TRADIER_PAPER_TOKEN"] = "UqYAFhBY0sPWSP4OmuAHChHB0lAN"
import sys
p = subprocess.Popen(
    [sys.executable, "-m", "uvicorn",
     "atlas_code_quant.api.main:app",
     "--host", "0.0.0.0", "--port", "8795", "--log-level", "info"],
    cwd=r"C:\ATLAS_PUSH",
    stdout=open(r"C:\ATLAS_PUSHpi_stdout2.log", "w"),
    stderr=open(r"C:\ATLAS_PUSHpi_stderr2.log", "w"),
    env={**os.environ},
    creationflags=0x00000008
)
print("Launched PID:", p.pid)
