#!/bin/bash

# Colores dorados y naranjas para el efecto mágico
GOLD='\033[38;5;220m'
ORANGE='\033[38;5;208m'
YELLOW='\033[1;33m'
BRIGHT_YELLOW='\033[38;5;226m'
DIM_GOLD='\033[38;5;178m'
NC='\033[0m'

# Obtener dimensiones de la terminal
COLS=$(tput cols)
ROWS=$(tput lines)
CENTER_X=$((COLS / 2))
CENTER_Y=$((ROWS / 2))

# Símbolos místicos y runas
SYMBOLS=("◈" "◉" "◎" "⬟" "⬢" "⬡" "⟐" "⟡" "⟢" "⟣" "◇" "◆" "⬙" "⬘" "▣" "▢" "⟴" "⟵" "⟶" "⟷")
RUNES=("ᚠ" "ᚢ" "ᚦ" "ᚨ" "ᚱ" "ᚲ" "ᚷ" "ᚹ" "ᚺ" "ᚾ" "ᛁ" "ᛃ" "ᛇ" "ᛈ" "ᛉ" "ᛊ" "ᛏ" "ᛒ" "ᛖ" "ᛗ")

# Función para limpiar pantalla
clear_screen() {
    clear
    tput civis  # Ocultar cursor
}

# Función para mover cursor a posición
move_cursor() {
    printf "\033[${1};${2}H"
}

# Función para dibujar círculo con símbolos
draw_circle() {
    local radius=$1
    local rotation=$2
    local color=$3
    local symbol_set=$4
    
    for ((angle=0; angle<360; angle+=15)); do
        local rad_angle=$(echo "scale=4; ($angle + $rotation) * 3.14159 / 180" | bc -l)
        local x=$(echo "scale=0; $CENTER_X + $radius * c($rad_angle)" | bc -l)
        local y=$(echo "scale=0; $CENTER_Y + $radius * s($rad_angle) / 2" | bc -l)
        
        # Verificar que esté dentro de los límites
        if [[ $x -gt 0 && $x -lt $COLS && $y -gt 0 && $y -lt $ROWS ]]; then
            move_cursor $y $x
            local symbol_index=$((angle / 15 % ${#symbol_set[@]}))
            printf "${color}${symbol_set[$symbol_index]}${NC}"
        fi
    done
}

# Función para dibujar líneas radiales
draw_radial_lines() {
    local rotation=$1
    local color=$2
    
    for ((angle=0; angle<360; angle+=30)); do
        local rad_angle=$(echo "scale=4; ($angle + $rotation) * 3.14159 / 180" | bc -l)
        
        for ((r=5; r<=15; r+=3)); do
            local x=$(echo "scale=0; $CENTER_X + $r * c($rad_angle)" | bc -l)
            local y=$(echo "scale=0; $CENTER_Y + $r * s($rad_angle) / 2" | bc -l)
            
            if [[ $x -gt 0 && $x -lt $COLS && $y -gt 0 && $y -lt $ROWS ]]; then
                move_cursor $y $x
                printf "${color}─${NC}"
            fi
        done
    done
}

# Función para dibujar el centro del portal
draw_center() {
    local pulse=$1
    local size=$((3 + pulse))
    
    for ((dy=-size; dy<=size; dy++)); do
        for ((dx=-size; dx<=size; dx++)); do
            local dist=$(echo "sqrt($dx*$dx + $dy*$dy)" | bc -l)
            if (( $(echo "$dist <= $size" | bc -l) )); then
                local x=$((CENTER_X + dx))
                local y=$((CENTER_Y + dy))
                
                if [[ $x -gt 0 && $x -lt $COLS && $y -gt 0 && $y -lt $ROWS ]]; then
                    move_cursor $y $x
                    if (( $(echo "$dist <= 1" | bc -l) )); then
                        printf "${BRIGHT_YELLOW}●${NC}"
                    elif (( $(echo "$dist <= 2" | bc -l) )); then
                        printf "${GOLD}◉${NC}"
                    else
                        printf "${ORANGE}◎${NC}"
                    fi
                fi
            fi
        done
    done
}

# Función para crear efecto de partículas
draw_particles() {
    local frame=$1
    
    for ((i=0; i<20; i++)); do
        local x=$(( (RANDOM % COLS) ))
        local y=$(( (RANDOM % ROWS) ))
        local particle_frame=$(( (frame + i) % 4 ))
        
        move_cursor $y $x
        case $particle_frame in
            0) printf "${DIM_GOLD}·${NC}" ;;
            1) printf "${GOLD}·${NC}" ;;
            2) printf "${BRIGHT_YELLOW}✦${NC}" ;;
            3) printf "${ORANGE}✧${NC}" ;;
        esac
    done
}

# Función principal de animación
animate_portal() {
    local frame=0
    
    while true; do
        clear_screen
        
        # Dibujar partículas de fondo
        draw_particles $frame
        
        # Círculo exterior (rotación lenta)
        draw_circle 20 $((frame * 2)) "$DIM_GOLD" SYMBOLS
        
        # Círculo medio (rotación opuesta)
        draw_circle 15 $((-frame * 3)) "$GOLD" RUNES
        
        # Círculo interior (rotación rápida)
        draw_circle 10 $((frame * 5)) "$ORANGE" SYMBOLS
        
        # Líneas radiales
        draw_radial_lines $((frame * 2)) "$YELLOW"
        
        # Centro pulsante
        local pulse=$(echo "scale=0; 2 * s($frame * 0.3)" | bc -l | sed 's/-//')
        draw_center $pulse
        
        # Texto central
        move_cursor $((CENTER_Y + 8)) $((CENTER_X - 10))
        printf "${BRIGHT_YELLOW}✦ PORTAL ACTIVADO ✦${NC}"
        
        # Incrementar frame
        frame=$((frame + 1))
        if [ $frame -gt 360 ]; then
            frame=0
        fi
        
        sleep 0.1
    done
}

# Función para manejar la salida limpia
cleanup() {
    tput cnorm  # Mostrar cursor
    clear
    exit 0
}

# Configurar trap para salida limpia
trap cleanup SIGINT SIGTERM

# Verificar si bc está instalado
if ! command -v bc &> /dev/null; then
    echo "Error: 'bc' no está instalado. Instálalo con: sudo apt install bc"
    exit 1
fi

# Mensaje inicial
echo -e "${BRIGHT_YELLOW}Presiona Ctrl+C para salir${NC}"
sleep 2

# Iniciar animación
animate_portal
