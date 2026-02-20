#!/usr/bin/env python3
"""Apply minimal inline IAM policy for Bedrock invoke to current IAM user."""
from __future__ import annotations

import json
import os
from pathlib import Path

import boto3


def load_env_from_txt(path: Path) -> None:
    if not path.exists():
        return
    allowed = {
        "AWS_ACCESS_KEY_ID",
        "AWS_SECRET_ACCESS_KEY",
        "AWS_REGION",
        "ATLAS_AI_MODE",
        "ANTHROPIC_API_KEY",
    }
    for raw in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        k = k.strip()
        v = v.strip().strip('"').strip("'")
        if k in allowed and v:
            os.environ[k] = v


def main() -> int:
    load_env_from_txt(Path(r"C:\dev\credenciales.txt"))
    region = os.getenv("AWS_REGION", "us-east-1")

    sts = boto3.client("sts", region_name=region)
    ident = sts.get_caller_identity()
    arn = ident.get("Arn", "")
    if ":user/" not in arn:
        print(f"Unsupported principal for inline user policy: {arn}")
        return 2
    user_name = arn.rsplit("/", 1)[-1]

    policy = {
        "Version": "2012-10-17",
        "Statement": [
            {
                "Sid": "AllowBedrockInvokeAnthropic",
                "Effect": "Allow",
                "Action": [
                    "bedrock:InvokeModel",
                    "bedrock:InvokeModelWithResponseStream",
                ],
                "Resource": [
                    "arn:aws:bedrock:us-east-1:*:inference-profile/us.anthropic.claude-opus-4-6-v1:0",
                    "arn:aws:bedrock:us-east-1:*:inference-profile/us.anthropic.claude-haiku-4-5-20251001-v1:0",
                ],
            }
        ],
    }

    iam = boto3.client("iam", region_name=region)
    iam.put_user_policy(
        UserName=user_name,
        PolicyName="ATLASBedrockInvokePolicy",
        PolicyDocument=json.dumps(policy),
    )
    print(json.dumps({"ok": True, "user": user_name, "policy_name": "ATLASBedrockInvokePolicy"}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
