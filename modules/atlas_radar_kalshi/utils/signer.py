"""
signer.py — Firmador RSA-PSS para Kalshi v2.

Implementa el esquema oficial documentado en
https://docs.kalshi.com/getting_started/api_keys :

1. ``timestamp`` (ms) + ``METHOD`` + ``path_sin_query``.
2. Firma con la llave privada RSA usando ``PSS(MGF1(SHA256),
   salt_length=DIGEST_LENGTH)``.
3. Codifica en base64 y la inyecta en el header
   ``KALSHI-ACCESS-SIGNATURE``.

Se mantiene como utilidad **independiente** del SDK oficial
``kalshi-python`` para poder reutilizarlo desde el WebSocket (que
firma el path ``/trade-api/ws/v2``) sin acoplarse a su implementación.
"""
from __future__ import annotations

import base64
import time
from pathlib import Path
from typing import Tuple
from urllib.parse import urlparse

from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import padding, rsa


class KalshiSigner:
    """Encapsula la firma RSA-PSS y la generación de headers."""

    def __init__(self, api_key_id: str, private_key_path: Path) -> None:
        self.api_key_id = api_key_id
        self._private_key = self._load_pem(private_key_path)

    # ------------------------------------------------------------------
    @staticmethod
    def _load_pem(path: Path) -> rsa.RSAPrivateKey:
        if not path.exists():
            raise FileNotFoundError(
                f"No se encontró la llave RSA privada en: {path}"
            )
        with open(path, "rb") as f:
            key = serialization.load_pem_private_key(
                f.read(), password=None, backend=default_backend()
            )
        if not isinstance(key, rsa.RSAPrivateKey):
            raise TypeError("La llave provista no es RSA.")
        return key

    # ------------------------------------------------------------------
    def _sign(self, message: str) -> str:
        sig = self._private_key.sign(
            message.encode("utf-8"),
            padding.PSS(
                mgf=padding.MGF1(hashes.SHA256()),
                salt_length=padding.PSS.DIGEST_LENGTH,
            ),
            hashes.SHA256(),
        )
        return base64.b64encode(sig).decode("utf-8")

    # ------------------------------------------------------------------
    def headers(self, method: str, full_url_or_path: str) -> Tuple[dict, str]:
        """
        Genera los headers requeridos por Kalshi v2.

        Parameters
        ----------
        method : str
            Verbo HTTP en mayúsculas ("GET", "POST", ...).
        full_url_or_path : str
            URL completa (``https://...`` o ``wss://...``) o path
            absoluto (``/trade-api/v2/...``). Se firma siempre el path
            sin query string.
        """
        parsed = urlparse(full_url_or_path)
        path = parsed.path or full_url_or_path
        path_no_query = path.split("?")[0]

        ts_ms = str(int(time.time() * 1000))
        message = f"{ts_ms}{method.upper()}{path_no_query}"
        signature = self._sign(message)

        headers = {
            "KALSHI-ACCESS-KEY": self.api_key_id,
            "KALSHI-ACCESS-SIGNATURE": signature,
            "KALSHI-ACCESS-TIMESTAMP": ts_ms,
            "Content-Type": "application/json",
        }
        return headers, ts_ms
