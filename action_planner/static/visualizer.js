// Path Planner Visualizer - JavaScript
// Gerencia a visualização interativa do grid e histórico

class PathPlannerVisualizer {
    constructor() {
        this.canvas = document.getElementById('gridCanvas');
        this.ctx = this.canvas.getContext('2d');
        this.gridData = null;
        this.currentPath = null;
        this.history = [];
        this.selectedHistoryId = null;
        
        this.cellSize = 50; // pixels por célula
        this.padding = 10;
        
        this.colors = {
            free: '#ecf0f1',
            obstacle: '#34495e',
            origin: '#3498db',
            destination: '#e74c3c',
            path: '#2ecc71',
            location: '#f39c12',
            grid: '#bdc3c7',
            text: '#2c3e50'
        };
        
        this.init();
    }
    
    async init() {
        await this.loadGridData();
        this.setupEventListeners();
        this.startPolling();
        this.updateStatus('connected', 'Conectado');
    }
    
    setupEventListeners() {
        document.getElementById('clearHistory').addEventListener('click', () => {
            this.selectedHistoryId = null;
            this.currentPath = null;
            this.drawGrid();
            document.getElementById('current-path-info').textContent = 'Aguardando planejamento...';
        });
    }
    
    async loadGridData() {
        try {
            const response = await fetch('/api/grid');
            this.gridData = await response.json();
            this.initCanvas();
            this.drawGrid();
        } catch (error) {
            console.error('Erro ao carregar dados do grid:', error);
            this.updateStatus('error', 'Erro ao conectar');
        }
    }
    
    initCanvas() {
        const rows = this.gridData.grid_size.rows;
        const cols = this.gridData.grid_size.cols;
        
        // Aumenta área de exibição para evitar corte dos nomes dos locais
        this.canvas.width = cols * this.cellSize + this.padding * 2;
        this.canvas.height = rows * this.cellSize + this.padding * 2 + 30; // +30px extra na vertical
    }
    
    drawGrid(pathData = null) {
        if (!this.gridData) return;
        
        const ctx = this.ctx;
        const grid = this.gridData.grid_map;
        const rows = grid.length;
        const cols = grid[0].length;
        
        // Limpa o canvas
        ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        
        // Desenha as células do grid
        for (let y = 0; y < rows; y++) {
            for (let x = 0; x < cols; x++) {
                const cellX = this.padding + x * this.cellSize;
                const cellY = this.padding + y * this.cellSize;
                
                // Cor da célula
                ctx.fillStyle = grid[y][x] === 1 ? this.colors.obstacle : this.colors.free;
                ctx.fillRect(cellX, cellY, this.cellSize, this.cellSize);
                
                // Borda da célula
                ctx.strokeStyle = this.colors.grid;
                ctx.strokeRect(cellX, cellY, this.cellSize, this.cellSize);
                
                // Coordenadas - ajustadas para ficarem mais à direita dentro do quadrado
                ctx.fillStyle = '#000000'; // Preto para melhor contraste
                ctx.font = '10px sans-serif';
                ctx.fillText(`${x},${y}`, cellX + 12, cellY + 12);
            }
        }
        
        // Desenha locais conhecidos
        for (const [name, coords] of Object.entries(this.gridData.location_coords)) {
            this.drawLocation(coords[0], coords[1], name);
        }
        
        // Desenha o caminho se houver
        if (pathData) {
            this.drawPath(pathData);
        }
    }
    
    drawLocation(x, y, name) {
        const ctx = this.ctx;
        const centerX = this.padding + x * this.cellSize + this.cellSize / 2;
        const centerY = this.padding + y * this.cellSize + this.cellSize / 2;
        
        // Círculo do local
        ctx.beginPath();
        ctx.arc(centerX, centerY, 8, 0, 2 * Math.PI);
        ctx.fillStyle = this.colors.location;
        ctx.fill();
        ctx.strokeStyle = '#d68910';
        ctx.lineWidth = 2;
        ctx.stroke();
        
        // Nome do local com borda branca para visibilidade
        // Posicionado mais próximo ao círculo para ficar dentro do quadrado
        ctx.font = 'bold 11px sans-serif';
        ctx.textAlign = 'center';
        
        // Desenha borda branca
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 3;
        ctx.strokeText(name, centerX, centerY + 18);
        
        // Desenha texto preto por cima
        ctx.fillStyle = '#000000';
        ctx.fillText(name, centerX, centerY + 18);
    }
    
    drawPath(pathData) {
        if (!pathData.path || pathData.path.length === 0) return;
        
        const ctx = this.ctx;
        const path = pathData.path;
        
        // Desenha linha do caminho
        ctx.strokeStyle = this.colors.path;
        ctx.lineWidth = 4;
        ctx.lineCap = 'round';
        ctx.lineJoin = 'round';
        
        ctx.beginPath();
        for (let i = 0; i < path.length; i++) {
            const [x, y] = path[i];
            const centerX = this.padding + x * this.cellSize + this.cellSize / 2;
            const centerY = this.padding + y * this.cellSize + this.cellSize / 2;
            
            if (i === 0) {
                ctx.moveTo(centerX, centerY);
            } else {
                ctx.lineTo(centerX, centerY);
            }
        }
        ctx.stroke();
        
        // Desenha pontos intermediários
        for (let i = 1; i < path.length - 1; i++) {
            const [x, y] = path[i];
            const centerX = this.padding + x * this.cellSize + this.cellSize / 2;
            const centerY = this.padding + y * this.cellSize + this.cellSize / 2;
            
            ctx.beginPath();
            ctx.arc(centerX, centerY, 5, 0, 2 * Math.PI);
            ctx.fillStyle = this.colors.path;
            ctx.fill();
        }
        
        // Destaca origem
        if (path.length > 0) {
            const [x, y] = path[0];
            const centerX = this.padding + x * this.cellSize + this.cellSize / 2;
            const centerY = this.padding + y * this.cellSize + this.cellSize / 2;
            
            ctx.beginPath();
            ctx.arc(centerX, centerY, 10, 0, 2 * Math.PI);
            ctx.fillStyle = this.colors.origin;
            ctx.fill();
            ctx.strokeStyle = '#2980b9';
            ctx.lineWidth = 2;
            ctx.stroke();
        }
        
        // Destaca destino
        if (path.length > 1) {
            const [x, y] = path[path.length - 1];
            const centerX = this.padding + x * this.cellSize + this.cellSize / 2;
            const centerY = this.padding + y * this.cellSize + this.cellSize / 2;
            
            ctx.beginPath();
            ctx.arc(centerX, centerY, 10, 0, 2 * Math.PI);
            ctx.fillStyle = this.colors.destination;
            ctx.fill();
            ctx.strokeStyle = '#c0392b';
            ctx.lineWidth = 2;
            ctx.stroke();
        }
    }
    
    async updateHistory() {
        try {
            const response = await fetch('/api/history');
            const history = await response.json();
            
            // Verifica se há novos itens
            if (history.length > this.history.length) {
                const latest = history[history.length - 1];
                this.currentPath = latest;
                this.selectedHistoryId = latest.id;
                this.drawGrid(latest);
                this.updateCurrentPathInfo(latest);
            }
            
            this.history = history;
            this.renderHistory();
        } catch (error) {
            console.error('Erro ao atualizar histórico:', error);
        }
    }
    
    renderHistory() {
        const historyList = document.getElementById('historyList');
        
        if (this.history.length === 0) {
            historyList.innerHTML = '<p class="empty-message">Nenhum planejamento realizado ainda.</p>';
            return;
        }
        
        // Renderiza em ordem reversa (mais recente primeiro)
        const reversedHistory = [...this.history].reverse();
        
        historyList.innerHTML = reversedHistory.map(item => {
            const time = new Date(item.timestamp).toLocaleTimeString('pt-BR');
            const statusClass = item.success ? 'success' : 'error';
            const selected = item.id === this.selectedHistoryId ? 'selected' : '';
            
            return `
                <div class="history-item ${statusClass} ${selected}" data-id="${item.id}">
                    <div class="history-header">
                        <span class="history-route">${item.origem} → ${item.destino}</span>
                        <span class="history-time">${time}</span>
                    </div>
                    <div class="history-details">
                        ${item.success ? 
                            `✓ Caminho encontrado: ${item.path_length} pontos` : 
                            `✗ ${item.message}`
                        }
                    </div>
                    <span class="history-status ${statusClass}">
                        ${item.success ? 'Sucesso' : 'Falha'}
                    </span>
                </div>
            `;
        }).join('');
        
        // Adiciona listeners para os itens
        historyList.querySelectorAll('.history-item').forEach(item => {
            item.addEventListener('click', (e) => {
                const id = parseInt(e.currentTarget.getAttribute('data-id'));
                this.selectHistoryItem(id);
            });
        });
    }
    
    selectHistoryItem(id) {
        const item = this.history.find(h => h.id === id);
        if (!item) return;
        
        this.selectedHistoryId = id;
        this.currentPath = item;
        this.drawGrid(item);
        this.updateCurrentPathInfo(item);
        this.renderHistory(); // Re-renderiza para atualizar seleção
    }
    
    updateCurrentPathInfo(pathData) {
        const info = document.getElementById('current-path-info');
        if (pathData.success) {
            info.textContent = `${pathData.origem} → ${pathData.destino} | ${pathData.path_length} pontos | ${pathData.message}`;
            info.style.color = '#27ae60';
        } else {
            info.textContent = `${pathData.origem} → ${pathData.destino} | ${pathData.message}`;
            info.style.color = '#e74c3c';
        }
    }
    
    startPolling() {
        // Atualiza a cada 1 segundo
        setInterval(() => this.updateHistory(), 1000);
    }
    
    updateStatus(status, text) {
        const indicator = document.getElementById('status-indicator');
        const statusText = document.getElementById('status-text');
        
        indicator.className = `status-dot ${status}`;
        statusText.textContent = text;
    }
}

// Inicializa quando o DOM estiver pronto
document.addEventListener('DOMContentLoaded', () => {
    new PathPlannerVisualizer();
});
