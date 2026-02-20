#!/usr/bin/env python3
"""Diagnostico rapido de conectividad AWS Bedrock para ATLAS AI Consultant."""
from __future__ import annotations

import asyncio
import json
import os
import sys
from typing import Any, Dict


def _mask(value: str) -> str:
    if not value:
        return ""
    if len(value) <= 6:
        return "***"
    return value[:3] + "***" + value[-3:]


def _env_snapshot() -> Dict[str, Any]:
    region = (os.getenv("AWS_REGION", "") or "").strip()
    mode = (os.getenv("ATLAS_AI_MODE", "") or "").strip().lower()
    access = os.getenv("AWS_ACCESS_KEY_ID", "")
    secret = os.getenv("AWS_SECRET_ACCESS_KEY", "")
    return {
        "aws_region": region,
        "atlas_ai_mode": mode,
        "aws_access_key_id": _mask(access),
        "aws_secret_access_key": _mask(secret),
        "has_access_key": bool(access),
        "has_secret_key": bool(secret),
    }


def check_sts_identity(region: str) -> Dict[str, Any]:
    import boto3

    sts = boto3.client("sts", region_name=region)
    identity = sts.get_caller_identity()
    return {
        "account": identity.get("Account"),
        "arn": identity.get("Arn"),
        "user_id": identity.get("UserId"),
    }


async def check_bedrock_anthropic(region: str, model_id: str) -> Dict[str, Any]:
    from anthropic import AsyncAnthropicBedrock

    client = AsyncAnthropicBedrock(aws_region=region)
    msg = await client.messages.create(
        model=model_id,
        max_tokens=32,
        temperature=0,
        messages=[{"role": "user", "content": "Responde SOLO: OK"}],
    )
    text_parts = []
    for block in getattr(msg, "content", []) or []:
        text = getattr(block, "text", None)
        if text:
            text_parts.append(text)
    return {
        "model": model_id,
        "response": "\n".join(text_parts).strip(),
    }


def main() -> int:
    env = _env_snapshot()
    print("[verify_bedrock] ENV =>")
    print(json.dumps(env, indent=2, ensure_ascii=False))

    region = env.get("aws_region") or "us-east-1"
    model_id = os.getenv("ATLAS_BEDROCK_FAST_MODEL", "us.anthropic.claude-haiku-4-5-20251001-v1:0")

    try:
        sts = check_sts_identity(region)
        print("[verify_bedrock] STS OK =>")
        print(json.dumps(sts, indent=2, ensure_ascii=False))
    except Exception as e:
        print(f"[verify_bedrock] STS ERROR: {e}")
        return 1

    try:
        result = asyncio.run(check_bedrock_anthropic(region, model_id))
        print("[verify_bedrock] BEDROCK OK =>")
        print(json.dumps(result, indent=2, ensure_ascii=False))
    except Exception as e:
        print(f"[verify_bedrock] BEDROCK ERROR: {e}")
        return 2

    print("[verify_bedrock] SUCCESS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
