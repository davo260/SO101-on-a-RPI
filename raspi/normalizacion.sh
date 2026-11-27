
#!/bin/bash

set -e

echo "=========================================="
echo "       NORMALIZACION DE PUERTOS SO101"
echo "=========================================="

# Verificar puertos fisicos
if [[ ! -e /dev/ttyACM0 ]]; then
    echo "[ERROR] No existe /dev/ttyACM0"
    exit 1
fi

if [[ ! -e /dev/ttyACM1 ]]; then
    echo "[ERROR] No existe /dev/ttyACM1"
    exit 1
fi

# Crear enlaces simbolicos directos
echo ""
echo "[1/2] Restaurando puertos fisicos..."

sudo ln -sf /dev/ttyACM0 /dev/so101_leader
sudo ln -sf /dev/ttyACM1 /dev/so101_follower

# Mostrar resultado
echo ""
echo "[2/2] Verificando enlaces..."

echo ""
echo "LEADER:"
ls -l /dev/so101_leader

echo ""
echo "FOLLOWER:"
ls -l /dev/so101_follower

echo ""
echo "=========================================="
echo "       NORMALIZACION COMPLETADA"
echo "=========================================="

