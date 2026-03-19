import { existsSync } from "node:fs";
import { spawnSync } from "node:child_process";

function textResult(text: string, details: Record<string, unknown> = {}) {
  return {
    content: [{ type: "text", text }],
    details,
  };
}

function normalizePath(value: unknown): string {
  return String(value || "").replace(/\//g, "\\");
}

function buildSummary(result: Record<string, any>): string {
  const command = result?.command || {};
  const validation = result?.validation || {};
  const execution = result?.execution || {};
  return [
    `status=${result?.status || "unknown"}`,
    `action=${command.action || "unknown"}`,
    `target=${command.target || "-"}`,
    `allowed=${validation.allowed === true ? "true" : "false"}`,
    `reason=${validation.reason || "-"}`,
    `backend_ok=${execution.ok === true ? "true" : "false"}`,
  ].join("\n");
}

const atlasRobotPlugin = {
  id: "atlas-robot-skill",
  name: "ATLAS Robot Skill",
  description: "Puente entre OpenClaw y el runtime de ATLAS.",
  register(api: any) {
    const cfg = {
      pythonPath: normalizePath(api.pluginConfig?.pythonPath || ""),
      skillConfig: normalizePath(api.pluginConfig?.skillConfig || ""),
      workspace: normalizePath(api.pluginConfig?.workspace || process.cwd()),
    };

    for (const key of ["pythonPath", "skillConfig", "workspace"]) {
      const value = cfg[key as keyof typeof cfg];
      if (!value) {
        api.logger.error(`atlas-robot-skill: missing config field ${key}`);
      }
    }

    api.logger.info(`atlas-robot-skill: registered using ${cfg.pythonPath}`);

    api.registerTool({
      name: "atlas_robot_control",
      label: "ATLAS Robot Control",
      description:
        "Ejecuta comandos naturales sobre ATLAS usando el skill Python con validacion de seguridad.",
      optional: true,
      parameters: {
        type: "object",
        additionalProperties: false,
        properties: {
          textCommand: {
            type: "string",
            description: "Orden natural, por ejemplo: ATLAS, mueve el brazo derecho 10 grados",
          },
          context: {
            type: "object",
            description: "Contexto adicional opcional para el skill.",
            additionalProperties: true,
          },
        },
        required: ["textCommand"],
      },
      async execute(_toolCallId: string, params: Record<string, unknown>) {
        const textCommand = String(params?.textCommand || "").trim();
        const context = (params?.context || {}) as Record<string, unknown>;

        if (!textCommand) {
          return textResult("Falta textCommand.", { ok: false });
        }

        for (const filePath of [cfg.pythonPath, cfg.skillConfig]) {
          if (!existsSync(filePath)) {
            return textResult(`Ruta no encontrada: ${filePath}`, {
              ok: false,
              missing: filePath,
            });
          }
        }

        const pyCode = [
          "import json, sys",
          "from modules.humanoid.openclaw import register",
          "config_path = sys.argv[1]",
          "command = sys.argv[2]",
          "context = json.loads(sys.argv[3])",
          "skill = register(config_path=config_path)",
          "print(json.dumps(skill.run(command, context=context), ensure_ascii=False))",
        ].join("; ");

        const proc = spawnSync(
          cfg.pythonPath,
          ["-c", pyCode, cfg.skillConfig, textCommand, JSON.stringify(context)],
          {
            cwd: cfg.workspace,
            encoding: "utf-8",
            timeout: 15000,
            env: { ...process.env, PYTHONIOENCODING: "utf-8" },
          },
        );

        if (proc.error) {
          return textResult(`Error lanzando skill Python: ${String(proc.error)}`, {
            ok: false,
            stderr: proc.stderr,
          });
        }

        if (proc.status !== 0) {
          return textResult(`Skill Python fallo con codigo ${proc.status}.`, {
            ok: false,
            stdout: proc.stdout,
            stderr: proc.stderr,
            status: proc.status,
          });
        }

        try {
          const parsed = JSON.parse(proc.stdout || "{}");
          return textResult(buildSummary(parsed), parsed);
        } catch (error) {
          return textResult("La salida del skill no fue JSON valido.", {
            ok: false,
            stdout: proc.stdout,
            stderr: proc.stderr,
            error: String(error),
          });
        }
      },
    } as any);
  },
};

export default atlasRobotPlugin;
