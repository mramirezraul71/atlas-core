@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "C:\ATLAS\ATLAS_VAULT\_SCRIPTS\atlas_note.ps1" -Text "%*" -Area "INBOX"
