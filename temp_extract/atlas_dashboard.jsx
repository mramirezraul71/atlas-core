import React, { useState, useEffect } from 'react';
import { Activity, Brain, Cpu, Database, Eye, Zap, TrendingUp, BookOpen, AlertCircle, CheckCircle, Clock, Layers } from 'lucide-react';

// ═══════════════════════════════════════════════════════════════
// ATLAS NEXUS DASHBOARD - PROFESSIONAL & MINIMALIST
// ═══════════════════════════════════════════════════════════════

const AtlasDashboard = () => {
  const [systemData, setSystemData] = useState({
    services: {
      push: { status: 'healthy', uptime: '47h 23m', port: 8791, load: 23 },
      nexus: { status: 'healthy', uptime: '47h 23m', port: 8000, load: 67 },
      robot: { status: 'healthy', uptime: '47h 23m', port: 8002, load: 45 }
    },
    learning: {
      currentLesson: 'Advanced Object Manipulation',
      progress: 68,
      tasksCompleted: 12,
      tasksTotal: 18,
      tutorScore: 85,
      uncertaintyRate: 0.23
    },
    memory: {
      episodic: { total: 1247, today: 34 },
      semantic: { concepts: 156, embeddings: 1247 },
      knowledgeBase: { concepts: 89, skills: 23, rules: 34 }
    },
    metrics: {
      successRate: 87.3,
      avgConfidence: 0.78,
      learningSpeed: 'medium-fast',
      consolidations: 12
    },
    recentActivity: [
      { time: '14:23', action: 'Identified fragile object', status: 'success', confidence: 0.92 },
      { time: '14:21', action: 'Consulted AI Tutor', status: 'success', confidence: 0.85 },
      { time: '14:18', action: 'Pick and place task', status: 'success', confidence: 0.88 },
      { time: '14:15', action: 'Visual classification', status: 'success', confidence: 0.91 },
      { time: '14:12', action: 'Uncertainty detected', status: 'warning', confidence: 0.45 }
    ]
  });

  // Simular actualizaciones en tiempo real
  useEffect(() => {
    const interval = setInterval(() => {
      setSystemData(prev => ({
        ...prev,
        services: {
          ...prev.services,
          push: { ...prev.services.push, load: Math.min(100, Math.max(10, prev.services.push.load + (Math.random() - 0.5) * 10)) },
          nexus: { ...prev.services.nexus, load: Math.min(100, Math.max(10, prev.services.nexus.load + (Math.random() - 0.5) * 10)) },
          robot: { ...prev.services.robot, load: Math.min(100, Math.max(10, prev.services.robot.load + (Math.random() - 0.5) * 10)) }
        }
      }));
    }, 3000);
    return () => clearInterval(interval);
  }, []);

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-950 via-slate-900 to-slate-950 text-gray-100 p-6">
      
      {/* ═══ HEADER ═══ */}
      <header className="mb-8">
        <div className="flex items-center justify-between">
          <div className="flex items-center space-x-4">
            <div className="w-12 h-12 bg-gradient-to-br from-cyan-500 to-blue-600 rounded-xl flex items-center justify-center shadow-lg shadow-cyan-500/20">
              <Brain className="w-7 h-7 text-white" />
            </div>
            <div>
              <h1 className="text-3xl font-bold bg-gradient-to-r from-cyan-400 to-blue-500 bg-clip-text text-transparent">
                ATLAS NEXUS
              </h1>
              <p className="text-sm text-gray-400 mt-1">Autonomous Learning & Tactical AI System</p>
            </div>
          </div>
          
          <div className="flex items-center space-x-4">
            <div className="px-4 py-2 bg-green-500/10 border border-green-500/20 rounded-lg">
              <div className="flex items-center space-x-2">
                <div className="w-2 h-2 bg-green-500 rounded-full animate-pulse"></div>
                <span className="text-sm font-medium text-green-400">All Systems Operational</span>
              </div>
            </div>
            <div className="text-right">
              <div className="text-xs text-gray-500">System Time</div>
              <div className="text-sm font-mono text-gray-300">
                {new Date().toLocaleTimeString()}
              </div>
            </div>
          </div>
        </div>
      </header>

      {/* ═══ MAIN GRID ═══ */}
      <div className="grid grid-cols-12 gap-6">
        
        {/* ═══ LEFT COLUMN - Services & Metrics ═══ */}
        <div className="col-span-8 space-y-6">
          
          {/* Services Status */}
          <div className="bg-slate-900/50 backdrop-blur border border-slate-800 rounded-2xl p-6 shadow-xl">
            <h2 className="text-lg font-semibold mb-4 flex items-center space-x-2">
              <Layers className="w-5 h-5 text-cyan-400" />
              <span>Core Services</span>
            </h2>
            
            <div className="grid grid-cols-3 gap-4">
              {Object.entries(systemData.services).map(([name, service]) => (
                <div key={name} className="bg-slate-800/50 rounded-xl p-4 border border-slate-700/50 hover:border-cyan-500/30 transition-all">
                  <div className="flex items-center justify-between mb-3">
                    <span className="text-sm font-medium text-gray-300 uppercase">{name}</span>
                    <div className="flex items-center space-x-1">
                      <div className="w-1.5 h-1.5 bg-green-500 rounded-full animate-pulse"></div>
                      <span className="text-xs text-green-400">Active</span>
                    </div>
                  </div>
                  
                  <div className="space-y-2">
                    <div className="flex justify-between text-xs">
                      <span className="text-gray-500">Port</span>
                      <span className="font-mono text-gray-300">{service.port}</span>
                    </div>
                    <div className="flex justify-between text-xs">
                      <span className="text-gray-500">Uptime</span>
                      <span className="font-mono text-gray-300">{service.uptime}</span>
                    </div>
                    
                    {/* Load Bar */}
                    <div>
                      <div className="flex justify-between text-xs mb-1">
                        <span className="text-gray-500">Load</span>
                        <span className="font-mono text-gray-300">{Math.round(service.load)}%</span>
                      </div>
                      <div className="h-1.5 bg-slate-700 rounded-full overflow-hidden">
                        <div 
                          className="h-full bg-gradient-to-r from-cyan-500 to-blue-500 transition-all duration-500"
                          style={{ width: `${service.load}%` }}
                        ></div>
                      </div>
                    </div>
                  </div>
                </div>
              ))}
            </div>
          </div>

          {/* Active Learning Session */}
          <div className="bg-gradient-to-br from-violet-900/20 to-fuchsia-900/20 backdrop-blur border border-violet-500/20 rounded-2xl p-6 shadow-xl">
            <div className="flex items-center justify-between mb-4">
              <h2 className="text-lg font-semibold flex items-center space-x-2">
                <BookOpen className="w-5 h-5 text-violet-400" />
                <span>Active Learning Session</span>
              </h2>
              <div className="px-3 py-1 bg-violet-500/20 border border-violet-500/30 rounded-full text-xs font-medium text-violet-300">
                Lesson in Progress
              </div>
            </div>
            
            <div className="mb-4">
              <div className="flex items-center justify-between mb-2">
                <h3 className="font-medium text-white">{systemData.learning.currentLesson}</h3>
                <span className="text-2xl font-bold text-violet-400">{systemData.learning.progress}%</span>
              </div>
              
              <div className="h-2 bg-slate-800 rounded-full overflow-hidden">
                <div 
                  className="h-full bg-gradient-to-r from-violet-500 via-fuchsia-500 to-pink-500 transition-all duration-500"
                  style={{ width: `${systemData.learning.progress}%` }}
                ></div>
              </div>
            </div>
            
            <div className="grid grid-cols-4 gap-4">
              <div className="text-center p-3 bg-slate-800/50 rounded-lg">
                <div className="text-2xl font-bold text-white">{systemData.learning.tasksCompleted}/{systemData.learning.tasksTotal}</div>
                <div className="text-xs text-gray-400 mt-1">Tasks Complete</div>
              </div>
              <div className="text-center p-3 bg-slate-800/50 rounded-lg">
                <div className="text-2xl font-bold text-emerald-400">{systemData.learning.tutorScore}</div>
                <div className="text-xs text-gray-400 mt-1">Tutor Score</div>
              </div>
              <div className="text-center p-3 bg-slate-800/50 rounded-lg">
                <div className="text-2xl font-bold text-amber-400">{(systemData.learning.uncertaintyRate * 100).toFixed(0)}%</div>
                <div className="text-xs text-gray-400 mt-1">Uncertainty</div>
              </div>
              <div className="text-center p-3 bg-slate-800/50 rounded-lg">
                <div className="text-2xl font-bold text-cyan-400">{systemData.metrics.learningSpeed}</div>
                <div className="text-xs text-gray-400 mt-1">Learning Speed</div>
              </div>
            </div>
          </div>

          {/* Performance Metrics */}
          <div className="grid grid-cols-4 gap-4">
            <MetricCard 
              icon={TrendingUp} 
              label="Success Rate" 
              value={`${systemData.metrics.successRate}%`}
              color="emerald"
              trend="+5.2%"
            />
            <MetricCard 
              icon={Zap} 
              label="Avg Confidence" 
              value={(systemData.metrics.avgConfidence * 100).toFixed(0) + '%'}
              color="cyan"
              trend="+2.1%"
            />
            <MetricCard 
              icon={Database} 
              label="Episodes Today" 
              value={systemData.memory.episodic.today}
              color="violet"
            />
            <MetricCard 
              icon={Brain} 
              label="Consolidations" 
              value={systemData.metrics.consolidations}
              color="fuchsia"
            />
          </div>

          {/* Recent Activity */}
          <div className="bg-slate-900/50 backdrop-blur border border-slate-800 rounded-2xl p-6 shadow-xl">
            <h2 className="text-lg font-semibold mb-4 flex items-center space-x-2">
              <Activity className="w-5 h-5 text-cyan-400" />
              <span>Recent Activity</span>
            </h2>
            
            <div className="space-y-2">
              {systemData.recentActivity.map((activity, idx) => (
                <div key={idx} className="flex items-center justify-between p-3 bg-slate-800/30 rounded-lg hover:bg-slate-800/50 transition-all border border-transparent hover:border-slate-700">
                  <div className="flex items-center space-x-3">
                    <div className="w-1.5 h-1.5 rounded-full" style={{
                      backgroundColor: activity.status === 'success' ? '#10b981' : activity.status === 'warning' ? '#f59e0b' : '#ef4444'
                    }}></div>
                    <span className="text-sm font-mono text-gray-500">{activity.time}</span>
                    <span className="text-sm text-gray-300">{activity.action}</span>
                  </div>
                  <div className="flex items-center space-x-2">
                    <span className="text-xs text-gray-500">confidence:</span>
                    <span className="text-sm font-mono text-cyan-400">{(activity.confidence * 100).toFixed(0)}%</span>
                  </div>
                </div>
              ))}
            </div>
          </div>
        </div>

        {/* ═══ RIGHT COLUMN - Memory & Knowledge ═══ */}
        <div className="col-span-4 space-y-6">
          
          {/* Memory Systems */}
          <div className="bg-slate-900/50 backdrop-blur border border-slate-800 rounded-2xl p-6 shadow-xl">
            <h2 className="text-lg font-semibold mb-4 flex items-center space-x-2">
              <Database className="w-5 h-5 text-cyan-400" />
              <span>Memory Systems</span>
            </h2>
            
            <div className="space-y-4">
              <MemoryCard 
                title="Episodic Memory"
                subtitle="SQLite Database"
                value={systemData.memory.episodic.total}
                subValue={`+${systemData.memory.episodic.today} today`}
                color="cyan"
              />
              
              <MemoryCard 
                title="Semantic Memory"
                subtitle="FAISS Embeddings"
                value={systemData.memory.semantic.embeddings}
                subValue={`${systemData.memory.semantic.concepts} concepts`}
                color="violet"
              />
              
              <MemoryCard 
                title="Knowledge Base"
                subtitle="Initial + Learned"
                value={systemData.memory.knowledgeBase.concepts}
                subValue={`${systemData.memory.knowledgeBase.skills} skills, ${systemData.memory.knowledgeBase.rules} rules`}
                color="fuchsia"
              />
            </div>
          </div>

          {/* AI Tutor Status */}
          <div className="bg-gradient-to-br from-emerald-900/20 to-teal-900/20 backdrop-blur border border-emerald-500/20 rounded-2xl p-6 shadow-xl">
            <h2 className="text-lg font-semibold mb-4 flex items-center space-x-2">
              <Brain className="w-5 h-5 text-emerald-400" />
              <span>AI Tutor</span>
            </h2>
            
            <div className="space-y-3">
              <div className="flex items-center justify-between">
                <span className="text-sm text-gray-400">Status</span>
                <div className="flex items-center space-x-2">
                  <div className="w-2 h-2 bg-emerald-500 rounded-full animate-pulse"></div>
                  <span className="text-sm font-medium text-emerald-400">Active</span>
                </div>
              </div>
              
              <div className="flex items-center justify-between">
                <span className="text-sm text-gray-400">Model</span>
                <span className="text-sm font-mono text-gray-300">Claude Opus 4.5</span>
              </div>
              
              <div className="flex items-center justify-between">
                <span className="text-sm text-gray-400">Lessons Completed</span>
                <span className="text-sm font-mono text-gray-300">23/45</span>
              </div>
              
              <div className="flex items-center justify-between">
                <span className="text-sm text-gray-400">Robot Level</span>
                <span className="text-sm font-medium text-emerald-400">Intermediate</span>
              </div>
              
              <div className="flex items-center justify-between">
                <span className="text-sm text-gray-400">Avg Score</span>
                <span className="text-sm font-bold text-white">85.3/100</span>
              </div>
              
              <div className="pt-3 border-t border-slate-700">
                <div className="text-xs text-gray-500 mb-2">Next Review In</div>
                <div className="text-2xl font-bold text-white font-mono">2h 34m</div>
              </div>
            </div>
          </div>

          {/* Vision System */}
          <div className="bg-slate-900/50 backdrop-blur border border-slate-800 rounded-2xl p-6 shadow-xl">
            <h2 className="text-lg font-semibold mb-4 flex items-center space-x-2">
              <Eye className="w-5 h-5 text-cyan-400" />
              <span>Vision System</span>
            </h2>
            
            <div className="space-y-3">
              <SystemStatus label="YOLO Detection" status="active" />
              <SystemStatus label="Depth Estimation (MiDaS)" status="active" />
              <SystemStatus label="Scene Understanding (LLaVA)" status="active" />
              <SystemStatus label="Multi-Camera Fusion" status="standby" />
            </div>
          </div>

          {/* System Health */}
          <div className="bg-slate-900/50 backdrop-blur border border-slate-800 rounded-2xl p-6 shadow-xl">
            <h2 className="text-lg font-semibold mb-4 flex items-center space-x-2">
              <Cpu className="w-5 h-5 text-cyan-400" />
              <span>System Health</span>
            </h2>
            
            <div className="space-y-3">
              <HealthBar label="CPU Usage" value={45} color="cyan" />
              <HealthBar label="Memory Usage" value={67} color="violet" />
              <HealthBar label="GPU Usage" value={82} color="fuchsia" />
              <HealthBar label="Storage" value={34} color="emerald" />
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

// ═══════════════════════════════════════════════════════════════
// COMPONENTS
// ═══════════════════════════════════════════════════════════════

const MetricCard = ({ icon: Icon, label, value, color, trend }) => {
  const colorClasses = {
    emerald: 'from-emerald-500 to-teal-500 text-emerald-400',
    cyan: 'from-cyan-500 to-blue-500 text-cyan-400',
    violet: 'from-violet-500 to-purple-500 text-violet-400',
    fuchsia: 'from-fuchsia-500 to-pink-500 text-fuchsia-400'
  };
  
  return (
    <div className="bg-slate-900/50 backdrop-blur border border-slate-800 rounded-xl p-4 shadow-lg hover:border-slate-700 transition-all">
      <div className={`w-10 h-10 bg-gradient-to-br ${colorClasses[color]} rounded-lg flex items-center justify-center mb-3 shadow-lg`}>
        <Icon className="w-5 h-5 text-white" />
      </div>
      <div className="text-2xl font-bold text-white mb-1">{value}</div>
      <div className="text-xs text-gray-400 mb-1">{label}</div>
      {trend && (
        <div className="text-xs text-emerald-400 font-medium">↑ {trend}</div>
      )}
    </div>
  );
};

const MemoryCard = ({ title, subtitle, value, subValue, color }) => {
  const colorClasses = {
    cyan: 'bg-cyan-500/10 border-cyan-500/20 text-cyan-400',
    violet: 'bg-violet-500/10 border-violet-500/20 text-violet-400',
    fuchsia: 'bg-fuchsia-500/10 border-fuchsia-500/20 text-fuchsia-400'
  };
  
  return (
    <div className={`${colorClasses[color]} border rounded-lg p-4`}>
      <div className="flex items-center justify-between mb-2">
        <h3 className="font-medium text-white">{title}</h3>
        <span className="text-2xl font-bold">{value}</span>
      </div>
      <div className="text-xs text-gray-400 mb-1">{subtitle}</div>
      <div className="text-xs opacity-70">{subValue}</div>
    </div>
  );
};

const SystemStatus = ({ label, status }) => {
  const statusConfig = {
    active: { color: 'bg-green-500', text: 'text-green-400', label: 'Active' },
    standby: { color: 'bg-amber-500', text: 'text-amber-400', label: 'Standby' },
    offline: { color: 'bg-red-500', text: 'text-red-400', label: 'Offline' }
  };
  
  const config = statusConfig[status];
  
  return (
    <div className="flex items-center justify-between p-3 bg-slate-800/30 rounded-lg">
      <span className="text-sm text-gray-300">{label}</span>
      <div className="flex items-center space-x-2">
        <div className={`w-1.5 h-1.5 ${config.color} rounded-full ${status === 'active' ? 'animate-pulse' : ''}`}></div>
        <span className={`text-xs ${config.text}`}>{config.label}</span>
      </div>
    </div>
  );
};

const HealthBar = ({ label, value, color }) => {
  const colorClasses = {
    cyan: 'from-cyan-500 to-blue-500',
    violet: 'from-violet-500 to-purple-500',
    fuchsia: 'from-fuchsia-500 to-pink-500',
    emerald: 'from-emerald-500 to-teal-500'
  };
  
  return (
    <div>
      <div className="flex justify-between text-xs mb-2">
        <span className="text-gray-400">{label}</span>
        <span className="font-mono text-gray-300">{value}%</span>
      </div>
      <div className="h-2 bg-slate-800 rounded-full overflow-hidden">
        <div 
          className={`h-full bg-gradient-to-r ${colorClasses[color]} transition-all duration-500`}
          style={{ width: `${value}%` }}
        ></div>
      </div>
    </div>
  );
};

export default AtlasDashboard;
