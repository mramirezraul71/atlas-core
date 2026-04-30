# 🚨 PROTOCOLO DE MANEJO DE ERRORES DE GIT

## 🎯 **POLÍTICA DE ERRORES DE GIT**

### **🚫 REGLA FUNDAMENTAL:**
**SI GITHUB DEVUELVE ERROR, NO SE PUEDE CONTINUAR SUBIENDO CON ERRORES EN CICLO REPETITIVO**

---

## 📋 **PROTOCOLO OFICIAL**

### **🚨 PASO 1: DETECCIÓN DE ERROR**
```bash
# Si git push devuelve error:
git push origin branch-name
# → ERROR: Repository rule violations / Push declined / etc.
```

**🛑 ACCIÓN INMEDIATA:**
1. **DETENER** todo intento de push
2. **NO CONTINUAR** con git push --force
3. **NO IGNORAR** el error
4. **DOCUMENTAR** el error exacto

---

### **📞 PASO 2: LLAMAR A ESPECIALISTA**

#### **👨‍💻 ESPECIALISTAS AUTORIZADOS:**
- **Admin de GitHub** con permisos de configuración
- **DevOps Engineer** con experiencia en Git
- **System Administrator** con acceso a repositorios

#### **📋 INFORMACIÓN REQUERIDA PARA ESPECIALISTA:**
```markdown
## Ticket de Error Git

**Fecha:** [Fecha y hora]
**Usuario:** [Usuario que intentó el push]
**Branch:** [nombre-del-branch]
**Hash del commit:** [hash-completo]
**Error exacto:** [copiar mensaje completo]
**Comandos ejecutados:** [lista de comandos]
**Contexto:** [qué se estaba haciendo]
```

---

### **🔧 PASO 3: ESPERAR RESOLUCIÓN**

#### **⏸️ MIENTRAS SE RESUELVE:**
- **NO HACER PUSH** de ningún cambio
- **TRABAJAR LOCALMENTE** si es necesario
- **DOCUMENTAR** cambios pendientes
- **CREAR BRANCHES LOCALES** si se necesita continuar trabajando

#### **📝 COMO DOCUMENTAR:**
```bash
# Crear branch local de trabajo
git checkout -b fix-pending-issues
# Continuar trabajando localmente
# NO hacer push hasta que se resuelva el error
```

---

## 🚨 **ESCENARIOS COMUNES Y ACCIONES**

### **🔒 ESCENARIO 1: Repository Rule Violations**
```
remote: error: GH013: Repository rule violations found
remote: - Push cannot contain secrets
```

**✅ ACCIÓN CORRECTA:**
1. **DETENER** inmediatamente
2. **NO USAR** --force
3. **LLAMAR** a especialista
4. **ELIMINAR** secretos del commit (requiere especialista)

**❌ ACCIONES PROHIBIDAS:**
- ❌ git push --force
- ❌ Ignorar el error
- ❌ Crear nuevo commit con los mismos secretos
- ❌ Hacer push a otro branch

---

### **🚫 ESCENARIO 2: Push Declined**
```
! [remote rejected] branch -> branch (push declined)
```

**✅ ACCIÓN CORRECTA:**
1. **DETENER** el push
2. **VERIFICAR** permisos del branch
3. **LLAMAR** a especialista para revisión
4. **ESPERAR** autorización

**❌ ACCIONES PROHIBIDAS:**
- ❌ git push --force
- ❌ Crear pull request sin permiso
- ❌ Bypass de protecciones

---

### **🔐 ESCENARIO 3: Authentication Issues**
```
remote: Authentication failed
```

**✅ ACCIÓN CORRECTA:**
1. **VERIFICAR** credenciales
2. **ROTAR** tokens si es necesario
3. **CONTACTAR** a especialista si persiste
4. **NO COMPARTIR** credenciales

---

## 🔄 **CICLO REPETITIVO - PROHIBIDO**

### **🚫 LO QUE NO SE DEBE HACER:**

```bash
# ❌ CICLO REPETITIVO INCORRECTO
git push origin branch  # ERROR
git push --force origin branch  # ERROR
git push origin branch  # ERROR NUEVAMENTE
git push --force origin branch  # ERROR NUEVAMENTE
# ... y así sucesivamente
```

### **✅ LO QUE SE DEBE HACER:**

```bash
# ✅ PROTOCOLO CORRECTO
git push origin branch  # ERROR
# DETENER INMEDIATAMENTE
# DOCUMENTAR ERROR
# LLAMAR A ESPECIALISTA
# ESPERAR RESOLUCIÓN
# RECIBIR AUTORIZACIÓN
# PROBAR SOLUCIÓN
# CONFIRMAR ÉXITO
```

---

## 📞 **CONTACTO DE ESPECIALISTAS**

### **👥 EQUIPO DE RESPUESTA:**

#### **🔧 Nivel 1 - Soporte Básico:**
- **DevOps Junior**: Errores de sintaxis
- **Git User**: Conflictos básicos

#### **👨‍💻 Nivel 2 - Especialistas:**
- **DevOps Senior**: Repository rules
- **System Admin**: Permisos y configuración
- **Security Engineer**: Secretos y tokens

#### **🚀 Nivel 3 - Expertos:**
- **GitHub Admin**: Configuración avanzada
- **Architect**: Políticas de repositorio

### **📋 ESCALAMIENTO:**
```
Error Detectado → Nivel 1 (5 min) → Si no resuelve → Nivel 2 (15 min) → Si no resuelve → Nivel 3 (30 min)
```

---

## 📊 **MONITOREO Y PREVENCIÓN**

### **🔍 ANTES DE HACER PUSH:**

#### **✅ CHECKLIST OBLIGATORIO:**
1. **Verificar archivos sensibles**: `git status`
2. **Revisar .gitignore**: Asegurar excludes
3. **Probar con --dry-run**: Si es posible
4. **Revisar mensaje de commit**: Sin información sensible
5. **Verificar branch correcto**: No hacer push a main/master sin protección

#### **🛡️ COMANDOS DE SEGURIDAD:**
```bash
# Verificar archivos antes de push
git status --porcelain
git diff --cached --name-only
git log --oneline -5

# Verificar excludes
cat .gitignore | grep -E "(key|secret|token|env)"
```

---

## 🎯 **RESPONSABILIDAD**

### **👤 RESPONSABLE DEL PROTOCOLO:**
- **Desarrollador**: Detectar y reportar errores
- **Especialista**: Resolver y documentar
- **Admin**: Supervisar y mejorar protocolo

### **📋 REGISTRO DE INCIDENTES:**
```markdown
## Registro de Error Git

**Fecha:** [Fecha]
**Error:** [Descripción]
**Especialista:** [Nombre]
**Resolución:** [Descripción]
**Tiempo:** [Duración]
**Lecciones aprendidas:** [Mejoras]
```

---

## 🚨 **ALERTAS DE EMERGENCIA**

### **🔥 SITUACIONES DE EMERGENCIA:**
1. **Datos sensibles expuestos** en el repositorio
2. **Branch principal corrupto**
3. **Pérdida de commits críticos**
4. **Acceso no autorizado detectado**

### **🚨 ACCIÓN INMEDIATA:**
1. **DETENER** toda actividad Git
2. **CONTACTAR** a admin inmediatamente
3. **DOCUMENTAR** situación completa
4. **SEGUIR** protocolo de emergencia

---

## 📋 **VERIFICACIÓN FINAL**

### **✅ CHECKLIST POST-RESOLUCIÓN:**
- [ ] Error completamente resuelto
- [ ] Push exitoso confirmado
- [ ] Branch sincronizado
- [ ] Documentación actualizada
- [ ] Lecciones aprendidas registradas
- [ ] Protocolo mejorado si es necesario

---

## 🎯 **MENSAJE CLAVE**

### **📢 REGLA DE ORO:**
> **"SI GITHUB DEVUELVE ERROR, DETENER Y LLAMAR A ESPECIALISTA. NUNCA HACER PUSH --FORCE SIN AUTORIZACIÓN."**

### **🔗 RECORDATORIO:**
- **La seguridad del repositorio es prioridad**
- **Los errores de Git son señales de alerta**
- **El ciclo repetitivo empeora los problemas**
- **Los especialistas existen para ayudar**

---

**🚨 ESTE PROTOCOLO ES OBLIGATORIO PARA TODOS LOS DESARROLLADORES DE ATLAS.**

**CUALQUIER VIOLACIÓN DE ESTE PROTOCOLO SERÁ CONSIDERADA UNA FALTA DE SEGURIDAD GRAVE.**
