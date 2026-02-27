# Instalación de Tesseract OCR

El módulo de visión/OCR requiere **Tesseract** (binario del sistema). No se instala con pip.

## Windows

```powershell
winget install UB-Mannheim.TesseractOCR
```

O descarga manual: https://github.com/UB-Mannheim/tesseract/wiki

Tras instalar, añade la ruta de `tesseract.exe` al PATH o configura `pytesseract.pytesseract.tesseract_cmd` en código.

## Verificar

```powershell
python -c "import pytesseract; print(pytesseract.get_tesseract_version())"
```

Si funciona, el ANS dejará de reportar `missing=['pytesseract','tesseract']`.
